#include "bluetooth_control.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

#define BLUETOOTH_RX_LINE_QUEUE_LENGTH 8U
#define BLUETOOTH_COMMAND_QUEUE_LENGTH 12U
#define BLUETOOTH_TX_LINE_QUEUE_LENGTH 64U
#define BLUETOOTH_TX_LINE_MAX          256U
#define BLUETOOTH_TX_TASK_STACK_WORDS  384U
#define BLUETOOTH_TX_TASK_PRIORITY     (tskIDLE_PRIORITY + 1U)
#define BLUETOOTH_TX_TIMEOUT_MS        30U
#define BLUETOOTH_RX_LINE_IDLE_MS      120U
#define BLUETOOTH_RETRY_RX_MS          1000U
#define BLUETOOTH_BT_MAP_ROW_INTERVAL_MS 0U
#define BLUETOOTH_BT_POSE_INTERVAL_MS    100U
#define BLUETOOTH_RX_ONLY_DEBUG        0U

typedef struct
{
  uint32_t tick_ms;
  char text[BLUETOOTH_LINE_MAX];
} BluetoothLine_t;

typedef struct
{
  UART_HandleTypeDef *huart;
  uint8_t rx_byte;
  char rx_line[BLUETOOTH_LINE_MAX];
  uint8_t rx_line_len;
  uint32_t last_byte_tick_ms;
} BluetoothRxContext_t;

typedef struct
{
  char text[BLUETOOTH_TX_LINE_MAX];
} BluetoothTxLine_t;

static StaticQueue_t s_rx_line_queue_struct;
static uint8_t s_rx_line_queue_storage[BLUETOOTH_RX_LINE_QUEUE_LENGTH * sizeof(BluetoothLine_t)];
static QueueHandle_t s_rx_line_queue;

static StaticQueue_t s_command_queue_struct;
static uint8_t s_command_queue_storage[BLUETOOTH_COMMAND_QUEUE_LENGTH * sizeof(BluetoothCommand_t)];
static QueueHandle_t s_command_queue;

static StaticQueue_t s_tx_line_queue_struct;
static uint8_t s_tx_line_queue_storage[BLUETOOTH_TX_LINE_QUEUE_LENGTH * sizeof(BluetoothTxLine_t)];
static QueueHandle_t s_tx_line_queue;

static StaticTask_t s_tx_task_struct;
static StackType_t s_tx_task_stack[BLUETOOTH_TX_TASK_STACK_WORDS];
static TaskHandle_t s_tx_task_handle;

static BluetoothControlState_t s_state;
static BluetoothRxContext_t s_rx_uart6;
static BluetoothRxContext_t s_rx_uart2;
static bool s_initialized;
static uint32_t s_last_rx_retry_tick_ms;
static uint32_t s_last_bt_map_row_tick_ms;
static uint32_t s_last_bt_pose_tick_ms;
static bool s_safe_set_capture_active;

static bool BluetoothControl_StartReceive(void);
static bool BluetoothControl_StartReceiveContext(BluetoothRxContext_t *context);
static BluetoothRxContext_t *BluetoothControl_GetRxContext(UART_HandleTypeDef *huart);
static bool BluetoothControl_SendBytesBlocking(const uint8_t *data, uint16_t length);
static bool BluetoothControl_ShouldSendToBluetooth(const uint8_t *data, uint16_t length);
static bool BluetoothControl_HasPrefix(const uint8_t *data, uint16_t length, const char *prefix);
static bool BluetoothControl_ShouldPreserveTxLine(const char *text);
static bool BluetoothControl_IsPriorityTxLine(const char *text);
static bool BluetoothControl_IsUrgentCommand(BluetoothCommandType_t command);
static void BluetoothControl_TxTask(void *argument);
static void BluetoothControl_FlushIdleRxLines(uint32_t now);
static void BluetoothControl_FlushIdleRxContext(BluetoothRxContext_t *context, uint32_t now);
static void BluetoothControl_QueueLine(const char *text, uint32_t tick_ms);
static void BluetoothControl_ProcessLine(const BluetoothLine_t *line);
static BluetoothCommandType_t BluetoothControl_ParseLine(const char *line);
static void BluetoothControl_NormalizeLine(const char *input, char *output, size_t output_size);
static void BluetoothControl_ApplyCommand(BluetoothCommandType_t command);
static void BluetoothControl_QueueCommand(BluetoothCommandType_t command, const char *text, uint32_t tick_ms);
static void BluetoothControl_SendAck(BluetoothCommandType_t command);
static void BluetoothControl_QueueLineFromIsr(const char *text, BaseType_t *higher_priority_task_woken);
static bool BluetoothControl_IsLineBreak(uint8_t byte);
static bool BluetoothControl_IsPrintable(uint8_t byte);
static bool BluetoothControl_IsUnsignedNumber(const char *line);
static bool BluetoothControl_IsSafeValueLine(const char *line);
static bool BluetoothControl_IsTurnDegreeCommand(const char *line, char prefix);
static char BluetoothControl_ToUpper(char c);

bool BluetoothControl_Init(void)
{
  if (s_initialized)
  {
    return s_state.ready;
  }

  memset(&s_state, 0, sizeof(s_state));
  s_safe_set_capture_active = false;

  s_rx_line_queue = xQueueCreateStatic(
      BLUETOOTH_RX_LINE_QUEUE_LENGTH,
      sizeof(BluetoothLine_t),
      s_rx_line_queue_storage,
      &s_rx_line_queue_struct);
  configASSERT(s_rx_line_queue != NULL);

  s_command_queue = xQueueCreateStatic(
      BLUETOOTH_COMMAND_QUEUE_LENGTH,
      sizeof(BluetoothCommand_t),
      s_command_queue_storage,
      &s_command_queue_struct);
  configASSERT(s_command_queue != NULL);

  s_tx_line_queue = xQueueCreateStatic(
      BLUETOOTH_TX_LINE_QUEUE_LENGTH,
      sizeof(BluetoothTxLine_t),
      s_tx_line_queue_storage,
      &s_tx_line_queue_struct);
  configASSERT(s_tx_line_queue != NULL);

  s_tx_task_handle = xTaskCreateStatic(
      BluetoothControl_TxTask,
      "btTx",
      BLUETOOTH_TX_TASK_STACK_WORDS,
      NULL,
      BLUETOOTH_TX_TASK_PRIORITY,
      s_tx_task_stack,
      &s_tx_task_struct);
  configASSERT(s_tx_task_handle != NULL);

  s_initialized = true;
  s_rx_uart6.huart = &huart6;
  s_rx_uart2.huart = &huart2;
  s_state.ready = BluetoothControl_StartReceive();
  return s_state.ready;
}

void BluetoothControl_Update(void)
{
  BluetoothLine_t line;
  uint32_t now = HAL_GetTick();

  if (!s_initialized)
  {
    (void)BluetoothControl_Init();
  }

  if (!s_state.ready && ((now - s_last_rx_retry_tick_ms) >= BLUETOOTH_RETRY_RX_MS))
  {
    s_last_rx_retry_tick_ms = now;
    s_state.ready = BluetoothControl_StartReceive();
  }

  while (xQueueReceive(s_rx_line_queue, &line, 0U) == pdPASS)
  {
    BluetoothControl_ProcessLine(&line);
  }

  BluetoothControl_FlushIdleRxLines(now);
}

bool BluetoothControl_TakeCommand(BluetoothCommand_t *out_command)
{
  if ((out_command == NULL) || (s_command_queue == NULL))
  {
    return false;
  }

  return (xQueueReceive(s_command_queue, out_command, 0U) == pdPASS);
}

bool BluetoothControl_GetState(BluetoothControlState_t *out_state)
{
  if (out_state == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *out_state = s_state;
  taskEXIT_CRITICAL();
  return s_initialized;
}

bool BluetoothControl_SendText(const char *text)
{
  size_t length;
  BluetoothTxLine_t tx_line;

  if (text == NULL)
  {
    return false;
  }

  length = strlen(text);
  if (length == 0U)
  {
    return true;
  }

  if (s_tx_line_queue == NULL)
  {
    return BluetoothControl_SendBytesBlocking((const uint8_t *)text, (uint16_t)length);
  }

  if (length >= sizeof(tx_line.text))
  {
    length = sizeof(tx_line.text) - 1U;
  }

  memcpy(tx_line.text, text, length);
  tx_line.text[length] = '\0';

  if (BluetoothControl_IsPriorityTxLine(tx_line.text))
  {
    if (xQueueSendToFront(s_tx_line_queue, &tx_line, 0U) == pdPASS)
    {
      return true;
    }

    {
      BluetoothTxLine_t dropped_line;

      (void)xQueueReceive(s_tx_line_queue, &dropped_line, 0U);
    }
    if (xQueueSendToFront(s_tx_line_queue, &tx_line, 0U) == pdPASS)
    {
      taskENTER_CRITICAL();
      s_state.tx_drops++;
      taskEXIT_CRITICAL();
      return true;
    }

    taskENTER_CRITICAL();
    s_state.tx_drops++;
    taskEXIT_CRITICAL();
    return false;
  }

  if (xQueueSend(s_tx_line_queue, &tx_line, 0U) != pdPASS)
  {
    if (BluetoothControl_ShouldPreserveTxLine(tx_line.text))
    {
      BluetoothTxLine_t dropped_line;

      (void)xQueueReceive(s_tx_line_queue, &dropped_line, 0U);
      if (xQueueSend(s_tx_line_queue, &tx_line, 0U) == pdPASS)
      {
        taskENTER_CRITICAL();
        s_state.tx_drops++;
        taskEXIT_CRITICAL();
        return true;
      }
    }

    taskENTER_CRITICAL();
    s_state.tx_drops++;
    taskEXIT_CRITICAL();
    return false;
  }

  return true;
}

static bool BluetoothControl_SendBytesBlocking(const uint8_t *data, uint16_t length)
{
  bool sent = false;

  if ((data == NULL) || (length == 0U))
  {
    return true;
  }

  if (BluetoothControl_ShouldSendToBluetooth(data, length) &&
      (HAL_UART_Transmit(&huart6, (uint8_t *)data, length, BLUETOOTH_TX_TIMEOUT_MS) == HAL_OK))
  {
    sent = true;
  }

  if (HAL_UART_Transmit(&huart2, (uint8_t *)data, length, BLUETOOTH_TX_TIMEOUT_MS) == HAL_OK)
  {
    sent = true;
  }

  if (sent)
  {
    s_state.tx_count++;
    s_state.last_tx_tick_ms = HAL_GetTick();
    return true;
  }

  s_state.ready = false;
  return false;
}

static bool BluetoothControl_ShouldSendToBluetooth(const uint8_t *data, uint16_t length)
{
  uint32_t now;

  if ((data == NULL) || (length == 0U))
  {
    return true;
  }

  if (BluetoothControl_HasPrefix(data, length, "ODOM "))
  {
    return false;
  }

  now = HAL_GetTick();
  if (BluetoothControl_HasPrefix(data, length, "MAP ROW "))
  {
    if ((BLUETOOTH_BT_MAP_ROW_INTERVAL_MS > 0U) &&
        ((now - s_last_bt_map_row_tick_ms) < BLUETOOTH_BT_MAP_ROW_INTERVAL_MS))
    {
      return false;
    }
    s_last_bt_map_row_tick_ms = now;
  }
  else if (BluetoothControl_HasPrefix(data, length, "POSE "))
  {
    if ((now - s_last_bt_pose_tick_ms) < BLUETOOTH_BT_POSE_INTERVAL_MS)
    {
      return false;
    }
    s_last_bt_pose_tick_ms = now;
  }

  return true;
}

static bool BluetoothControl_ShouldPreserveTxLine(const char *text)
{
  if (text == NULL)
  {
    return false;
  }

  return (strncmp(text, "MAP START ", 10U) == 0) ||
         (strncmp(text, "MAP SNAP ", 9U) == 0) ||
         (strncmp(text, "MAP STOP ", 9U) == 0) ||
         (strncmp(text, "MAP IDLE ", 9U) == 0) ||
         (strncmp(text, "MAP STAT ", 9U) == 0) ||
         (strncmp(text, "POSE ", 5U) == 0) ||
         (strncmp(text, "PATH ", 5U) == 0) ||
         (strncmp(text, "SLAM HB ", 8U) == 0);
}

static bool BluetoothControl_IsPriorityTxLine(const char *text)
{
  if (text == NULL)
  {
    return false;
  }

  return (strncmp(text, "ACK ", 4U) == 0) ||
         (strncmp(text, "ERR ", 4U) == 0) ||
         (strncmp(text, "EMERGENCY", 9U) == 0) ||
         (strncmp(text, "MOTOR ", 6U) == 0) ||
         (strncmp(text, "MODE86 ", 7U) == 0) ||
         (strncmp(text, "SLAM STOP", 9U) == 0) ||
         (strncmp(text, "MAP STOP", 8U) == 0) ||
         (strncmp(text, "GYRO CAL", 8U) == 0) ||
         (strncmp(text, "MPU STATE", 9U) == 0) ||
         (strncmp(text, "DIR ", 4U) == 0) ||
         (strncmp(text, "LIDAR QUALITY", 13U) == 0) ||
         (strncmp(text, "LIDAR FRONT", 11U) == 0);
}

static bool BluetoothControl_IsUrgentCommand(BluetoothCommandType_t command)
{
  return (command == BLUETOOTH_CMD_STOP_ALL) ||
         (command == BLUETOOTH_CMD_DRIVE_STOP) ||
         (command == BLUETOOTH_CMD_SLAM_NAV_OFF) ||
         (command == BLUETOOTH_CMD_MODE86_OFF) ||
         (command == BLUETOOTH_CMD_AUTO_MAPPING_OFF) ||
         (command == BLUETOOTH_CMD_STOP_MAPPING);
}

static bool BluetoothControl_HasPrefix(const uint8_t *data, uint16_t length, const char *prefix)
{
  size_t prefix_len;

  if ((data == NULL) || (prefix == NULL))
  {
    return false;
  }

  prefix_len = strlen(prefix);
  if (length < prefix_len)
  {
    return false;
  }

  return (strncmp((const char *)data, prefix, prefix_len) == 0);
}

static void BluetoothControl_TxTask(void *argument)
{
  BluetoothTxLine_t tx_line;

  (void)argument;

  for (;;)
  {
    if ((s_tx_line_queue != NULL) &&
        (xQueueReceive(s_tx_line_queue, &tx_line, portMAX_DELAY) == pdPASS))
    {
      (void)BluetoothControl_SendBytesBlocking(
          (const uint8_t *)tx_line.text,
          (uint16_t)strlen(tx_line.text));
    }
  }
}

const char *BluetoothControl_CommandName(BluetoothCommandType_t command)
{
  switch (command)
  {
    case BLUETOOTH_CMD_START_MAPPING:   return "START_MAPPING";
    case BLUETOOTH_CMD_STOP_MAPPING:    return "STOP_MAPPING";
    case BLUETOOTH_CMD_STOP_ALL:        return "STOP_ALL";
    case BLUETOOTH_CMD_SHOW_MAP_RESULT: return "SHOW_MAP";
    case BLUETOOTH_CMD_DEBUG_ON:        return "DEBUG_ON";
    case BLUETOOTH_CMD_DEBUG_OFF:       return "DEBUG_OFF";
    case BLUETOOTH_CMD_LIDAR_DEBUG_ON:  return "LIDAR_DEBUG_ON";
    case BLUETOOTH_CMD_LIDAR_DEBUG_OFF: return "LIDAR_DEBUG_OFF";
    case BLUETOOTH_CMD_LIDAR_QUALITY_SET:return "LIDAR_QUALITY_SET";
    case BLUETOOTH_CMD_SAFE_SET:        return "SAFE_SET";
    case BLUETOOTH_CMD_SAFE_VALUE:      return "SAFE_VALUE";
    case BLUETOOTH_CMD_SAFE_END:        return "SAFE_END";
    case BLUETOOTH_CMD_ODOM_DEBUG_ON:   return "ODOM_DEBUG_ON";
    case BLUETOOTH_CMD_ODOM_DEBUG_OFF:  return "ODOM_DEBUG_OFF";
    case BLUETOOTH_CMD_ENCODER_CAL_END: return "ENCODER_CAL_END";
    case BLUETOOTH_CMD_AUTO_MAPPING_ON: return "AUTO_MAPPING_ON";
    case BLUETOOTH_CMD_AUTO_MAPPING_OFF:return "AUTO_MAPPING_OFF";
    case BLUETOOTH_CMD_MODE86_ON:       return "MODE86_ON";
    case BLUETOOTH_CMD_MODE86_OFF:      return "MODE86_OFF";
    case BLUETOOTH_CMD_SLAM_NAV_ON:     return "SLAM_NAV_ON";
    case BLUETOOTH_CMD_SLAM_NAV_OFF:    return "SLAM_NAV_OFF";
    case BLUETOOTH_CMD_SLAM_NAV_RETURN: return "SLAM_NAV_RETURN";
    case BLUETOOTH_CMD_GYRO_CALIBRATE:  return "GYRO_CALIBRATE";
    case BLUETOOTH_CMD_MPU_STATE:       return "MPU_STATE";
    case BLUETOOTH_CMD_LIDAR_FRONT_STATE:return "LIDAR_FRONT_STATE";
    case BLUETOOTH_CMD_DIR_RESET:       return "DIR_RESET";
    case BLUETOOTH_CMD_DIR_STATE:       return "DIR_STATE";
    case BLUETOOTH_CMD_TURN_LEFT_DEG:   return "TURN_LEFT_DEG";
    case BLUETOOTH_CMD_TURN_RIGHT_DEG:  return "TURN_RIGHT_DEG";
    case BLUETOOTH_CMD_DRIVE_FORWARD:   return "DRIVE_FORWARD";
    case BLUETOOTH_CMD_TURN_LEFT:       return "TURN_LEFT";
    case BLUETOOTH_CMD_TURN_RIGHT:      return "TURN_RIGHT";
    case BLUETOOTH_CMD_DRIVE_STOP:      return "DRIVE_STOP";
    case BLUETOOTH_CMD_UNKNOWN:         return "UNKNOWN";
    case BLUETOOTH_CMD_NONE:
    default:                            return "NONE";
  }
}

void BluetoothControl_OnUartRxCpltFromIsr(UART_HandleTypeDef *huart)
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  BluetoothLine_t completed_line;
  BluetoothRxContext_t *context;
  uint8_t byte;

  context = BluetoothControl_GetRxContext(huart);
  if ((context == NULL) || (s_rx_line_queue == NULL))
  {
    return;
  }

  byte = context->rx_byte;
  s_state.rx_bytes++;
  s_state.last_rx_tick_ms = HAL_GetTick();
  context->last_byte_tick_ms = s_state.last_rx_tick_ms;

  if (BluetoothControl_IsLineBreak(byte))
  {
    if (context->rx_line_len > 0U)
    {
      context->rx_line[context->rx_line_len] = '\0';
      BluetoothControl_QueueLineFromIsr(context->rx_line, &higher_priority_task_woken);
      context->rx_line_len = 0U;
    }
  }
  else if (BluetoothControl_IsPrintable(byte))
  {
    if ((context->rx_line_len == 0U) &&
        ((byte == (uint8_t)'0') || (byte == (uint8_t)'2') || (byte == (uint8_t)'3')))
    {
      completed_line.text[0] = (char)byte;
      completed_line.text[1] = '\0';
      BluetoothControl_QueueLineFromIsr(completed_line.text, &higher_priority_task_woken);
    }
    else if ((context->rx_line_len == 1U) && (context->rx_line[0] == '9') && (byte == (uint8_t)'1'))
    {
      BluetoothControl_QueueLineFromIsr("91", &higher_priority_task_woken);
      context->rx_line_len = 0U;
    }
    else if (context->rx_line_len < (BLUETOOTH_LINE_MAX - 1U))
    {
      context->rx_line[context->rx_line_len++] = (char)byte;
    }
    else
    {
      context->rx_line_len = 0U;
      s_state.rx_overflows++;
    }
  }

  s_state.ready = BluetoothControl_StartReceiveContext(context);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void BluetoothControl_OnUartErrorFromIsr(UART_HandleTypeDef *huart)
{
  BluetoothRxContext_t *context = BluetoothControl_GetRxContext(huart);

  if (context == NULL)
  {
    return;
  }

  s_state.uart_errors++;
  s_state.ready = BluetoothControl_StartReceiveContext(context);
}

static bool BluetoothControl_StartReceive(void)
{
  bool ok = true;

  ok = BluetoothControl_StartReceiveContext(&s_rx_uart6) && ok;
  ok = BluetoothControl_StartReceiveContext(&s_rx_uart2) && ok;
  return ok;
}

static bool BluetoothControl_StartReceiveContext(BluetoothRxContext_t *context)
{
  if ((context == NULL) || (context->huart == NULL))
  {
    return false;
  }

  return (HAL_UART_Receive_IT(context->huart, &context->rx_byte, 1U) == HAL_OK);
}

static BluetoothRxContext_t *BluetoothControl_GetRxContext(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return NULL;
  }

  if (huart->Instance == USART6)
  {
    return &s_rx_uart6;
  }

  if (huart->Instance == USART2)
  {
    return &s_rx_uart2;
  }

  return NULL;
}

static void BluetoothControl_FlushIdleRxLines(uint32_t now)
{
  BluetoothControl_FlushIdleRxContext(&s_rx_uart6, now);
  BluetoothControl_FlushIdleRxContext(&s_rx_uart2, now);
}

static void BluetoothControl_FlushIdleRxContext(BluetoothRxContext_t *context, uint32_t now)
{
  char line[BLUETOOTH_LINE_MAX];
  uint8_t length;
  uint32_t tick_ms;

  if ((context == NULL) || (context->rx_line_len == 0U))
  {
    return;
  }

  if ((now - context->last_byte_tick_ms) < BLUETOOTH_RX_LINE_IDLE_MS)
  {
    return;
  }

  taskENTER_CRITICAL();
  if ((context->rx_line_len == 0U) ||
      ((now - context->last_byte_tick_ms) < BLUETOOTH_RX_LINE_IDLE_MS))
  {
    taskEXIT_CRITICAL();
    return;
  }

  length = context->rx_line_len;
  if (length >= BLUETOOTH_LINE_MAX)
  {
    length = BLUETOOTH_LINE_MAX - 1U;
  }
  memcpy(line, context->rx_line, length);
  line[length] = '\0';
  tick_ms = context->last_byte_tick_ms;
  context->rx_line_len = 0U;
  taskEXIT_CRITICAL();

  BluetoothControl_QueueLine(line, tick_ms);
}

static void BluetoothControl_QueueLine(const char *text, uint32_t tick_ms)
{
  BluetoothLine_t completed_line;
  BluetoothLine_t dropped_line;

  if ((text == NULL) || (s_rx_line_queue == NULL))
  {
    return;
  }

  completed_line.tick_ms = tick_ms;
  memset(completed_line.text, 0, sizeof(completed_line.text));
  strncpy(completed_line.text, text, sizeof(completed_line.text) - 1U);

  if (xQueueSend(s_rx_line_queue, &completed_line, 0U) != pdPASS)
  {
    (void)xQueueReceive(s_rx_line_queue, &dropped_line, 0U);
    if (xQueueSend(s_rx_line_queue, &completed_line, 0U) == pdPASS)
    {
      s_state.command_drops++;
      return;
    }

    s_state.command_drops++;
  }
}

static void BluetoothControl_ProcessLine(const BluetoothLine_t *line)
{
  char normalized[BLUETOOTH_LINE_MAX];
  BluetoothCommandType_t command;

  BluetoothControl_NormalizeLine(line->text, normalized, sizeof(normalized));
  command = BluetoothControl_ParseLine(normalized);

  strncpy(s_state.last_line, normalized, sizeof(s_state.last_line) - 1U);
  s_state.last_line[sizeof(s_state.last_line) - 1U] = '\0';
  s_state.rx_lines++;

#if (BLUETOOTH_RX_ONLY_DEBUG != 0U)
  s_state.last_command = BLUETOOTH_CMD_NONE;
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  return;
#endif

  if (command == BLUETOOTH_CMD_UNKNOWN)
  {
    s_state.parse_errors++;
    (void)BluetoothControl_SendText("ERR CMD\r\n");
    return;
  }

  if ((command != BLUETOOTH_CMD_SAFE_SET) &&
      (command != BLUETOOTH_CMD_SAFE_VALUE) &&
      (command != BLUETOOTH_CMD_SAFE_END))
  {
    s_safe_set_capture_active = false;
  }

  BluetoothControl_ApplyCommand(command);
  BluetoothControl_QueueCommand(command, normalized, line->tick_ms);
  BluetoothControl_SendAck(command);
}

static BluetoothCommandType_t BluetoothControl_ParseLine(const char *line)
{
  if ((strcmp(line, "SAFE SET") == 0) ||
      (strcmp(line, "SET SAFE") == 0))
  {
    s_safe_set_capture_active = true;
    return BLUETOOTH_CMD_SAFE_SET;
  }

  if ((strcmp(line, "SAFE END") == 0) ||
      (strcmp(line, "END SAFE") == 0))
  {
    s_safe_set_capture_active = false;
    return BLUETOOTH_CMD_SAFE_END;
  }

  if (s_safe_set_capture_active && BluetoothControl_IsUnsignedNumber(line))
  {
    if (strcmp(line, "0") == 0)
    {
      return BLUETOOTH_CMD_DRIVE_STOP;
    }
    return BLUETOOTH_CMD_SAFE_VALUE;
  }

  if ((strncmp(line, "SAFE ", 5U) == 0) && BluetoothControl_IsSafeValueLine(line))
  {
    return BLUETOOTH_CMD_SAFE_VALUE;
  }

  if (strcmp(line, "91") == 0)
  {
    return BLUETOOTH_CMD_START_MAPPING;
  }

  if (strcmp(line, "97") == 0)
  {
    return BLUETOOTH_CMD_STOP_MAPPING;
  }

  if (strcmp(line, "1") == 0)
  {
    return BLUETOOTH_CMD_DRIVE_FORWARD;
  }

  if (strcmp(line, "2") == 0)
  {
    return BLUETOOTH_CMD_TURN_LEFT;
  }

  if (strcmp(line, "3") == 0)
  {
    return BLUETOOTH_CMD_TURN_RIGHT;
  }

  if (strcmp(line, "0") == 0)
  {
    return BLUETOOTH_CMD_DRIVE_STOP;
  }

  if ((strcmp(line, "START") == 0) ||
      (strcmp(line, "MAP START") == 0) ||
      (strcmp(line, "START MAP") == 0))
  {
    return BLUETOOTH_CMD_START_MAPPING;
  }

  if ((strcmp(line, "MAP STOP") == 0) ||
      (strcmp(line, "STOP MAP") == 0) ||
      (strcmp(line, "MAP OFF") == 0))
  {
    return BLUETOOTH_CMD_STOP_MAPPING;
  }

  if ((strcmp(line, "EMERGENCY") == 0) ||
      (strcmp(line, "EMERGENCY STOP") == 0) ||
      (strcmp(line, "ESTOP") == 0) ||
      (strcmp(line, "E STOP") == 0) ||
      (strcmp(line, "ALL STOP") == 0) ||
      (strcmp(line, "STOP ALL") == 0))
  {
    return BLUETOOTH_CMD_STOP_ALL;
  }

  if ((strcmp(line, "STOP") == 0) ||
      (strcmp(line, "HALT") == 0))
  {
    return BLUETOOTH_CMD_DRIVE_STOP;
  }

  if ((strcmp(line, "SHOW") == 0) ||
      (strcmp(line, "RESULT") == 0) ||
      (strcmp(line, "MAP SHOW") == 0) ||
      (strcmp(line, "SHOW MAP") == 0))
  {
    return BLUETOOTH_CMD_SHOW_MAP_RESULT;
  }

  if ((strcmp(line, "DEBUG ON") == 0) ||
      (strcmp(line, "DBG ON") == 0))
  {
    return BLUETOOTH_CMD_DEBUG_ON;
  }

  if ((strcmp(line, "DEBUG OFF") == 0) ||
      (strcmp(line, "DBG OFF") == 0))
  {
    return BLUETOOTH_CMD_DEBUG_OFF;
  }

  if ((strcmp(line, "92") == 0) ||
      (strcmp(line, "LIDAR") == 0) ||
      (strcmp(line, "LIDAR ON") == 0) ||
      (strcmp(line, "LIDAR DEBUG") == 0) ||
      (strcmp(line, "LIDAR DEBUG ON") == 0))
  {
    return BLUETOOTH_CMD_LIDAR_DEBUG_ON;
  }

  if ((strcmp(line, "93") == 0) ||
      (strcmp(line, "LIDAR OFF") == 0) ||
      (strcmp(line, "LIDAR DEBUG OFF") == 0))
  {
    return BLUETOOTH_CMD_LIDAR_DEBUG_OFF;
  }

  if ((strncmp(line, "LIDAR QUALITY", 13U) == 0) ||
      (strncmp(line, "LIDAR Q", 7U) == 0) ||
      (strncmp(line, "QUALITY", 7U) == 0) ||
      (strncmp(line, "Q ", 2U) == 0))
  {
    return BLUETOOTH_CMD_LIDAR_QUALITY_SET;
  }

  if ((strcmp(line, "94") == 0) ||
      (strcmp(line, "ODOM") == 0) ||
      (strcmp(line, "ODOM ON") == 0) ||
      (strcmp(line, "ODOM DEBUG") == 0) ||
      (strcmp(line, "ODOM DEBUG ON") == 0))
  {
    return BLUETOOTH_CMD_ODOM_DEBUG_ON;
  }

  if ((strcmp(line, "95") == 0) ||
      (strcmp(line, "ODOM OFF") == 0) ||
      (strcmp(line, "ODOM DEBUG OFF") == 0))
  {
    return BLUETOOTH_CMD_ODOM_DEBUG_OFF;
  }

  if ((strcmp(line, "END ENCODER") == 0) ||
      (strcmp(line, "ENCODER END") == 0) ||
      (strcmp(line, "END ODOM") == 0) ||
      (strcmp(line, "ODOM END") == 0) ||
      (strcmp(line, "ENCODER CAL END") == 0))
  {
    return BLUETOOTH_CMD_ENCODER_CAL_END;
  }

  if ((strcmp(line, "96") == 0) ||
      (strcmp(line, "AUTO MAP") == 0) ||
      (strcmp(line, "AUTO MAP ON") == 0) ||
      (strcmp(line, "MAPPING AUTO") == 0) ||
      (strcmp(line, "MAPPING AUTO ON") == 0))
  {
    return BLUETOOTH_CMD_AUTO_MAPPING_ON;
  }

  if ((strcmp(line, "AUTO MAP OFF") == 0) ||
      (strcmp(line, "AUTO WALL OFF") == 0) ||
      (strcmp(line, "WALL OFF") == 0) ||
      (strcmp(line, "96 OFF") == 0) ||
      (strcmp(line, "MAPPING AUTO OFF") == 0))
  {
    return BLUETOOTH_CMD_AUTO_MAPPING_OFF;
  }

  if ((strcmp(line, "86") == 0) ||
      (strcmp(line, "86 ON") == 0) ||
      (strcmp(line, "MODE86") == 0) ||
      (strcmp(line, "MODE86 ON") == 0) ||
      (strcmp(line, "MODE 86") == 0) ||
      (strcmp(line, "MODE 86 ON") == 0) ||
      (strcmp(line, "AVOID SLAM") == 0) ||
      (strcmp(line, "AVOID SLAM ON") == 0))
  {
    return BLUETOOTH_CMD_MODE86_ON;
  }

  if ((strcmp(line, "86 OFF") == 0) ||
      (strcmp(line, "MODE86 OFF") == 0) ||
      (strcmp(line, "MODE 86 OFF") == 0) ||
      (strcmp(line, "AVOID SLAM OFF") == 0))
  {
    return BLUETOOTH_CMD_MODE86_OFF;
  }

  if ((strcmp(line, "98") == 0) ||
      (strcmp(line, "SLAM") == 0) ||
      (strcmp(line, "SLAM ON") == 0) ||
      (strcmp(line, "ASTAR") == 0) ||
      (strcmp(line, "ASTAR ON") == 0))
  {
    return BLUETOOTH_CMD_SLAM_NAV_ON;
  }

  if ((strcmp(line, "99") == 0) ||
      (strcmp(line, "SLAM OFF") == 0) ||
      (strcmp(line, "ASTAR OFF") == 0))
  {
    return BLUETOOTH_CMD_SLAM_NAV_OFF;
  }

  if ((strcmp(line, "100") == 0) ||
      (strcmp(line, "BACK") == 0) ||
      (strcmp(line, "RETURN") == 0) ||
      (strcmp(line, "HOME") == 0) ||
      (strcmp(line, "GO HOME") == 0) ||
      (strcmp(line, "SLAM BACK") == 0) ||
      (strcmp(line, "SLAM RETURN") == 0) ||
      (strcmp(line, "ASTAR BACK") == 0) ||
      (strcmp(line, "ASTAR RETURN") == 0))
  {
    return BLUETOOTH_CMD_SLAM_NAV_RETURN;
  }

  if ((strcmp(line, "CAL") == 0) ||
      (strcmp(line, "CAL MPU") == 0) ||
      (strcmp(line, "CAL GYRO") == 0) ||
      (strcmp(line, "GYRO CAL") == 0) ||
      (strcmp(line, "GYRO CALIBRATE") == 0) ||
      (strcmp(line, "IMU CAL") == 0) ||
      (strcmp(line, "MPU CAL") == 0) ||
      (strcmp(line, "MPU CALIBRATE") == 0))
  {
    return BLUETOOTH_CMD_GYRO_CALIBRATE;
  }

  if ((strcmp(line, "MPU") == 0) ||
      (strcmp(line, "MPU STATE") == 0) ||
      (strcmp(line, "IMU") == 0) ||
      (strcmp(line, "IMU STATE") == 0) ||
      (strcmp(line, "GYRO STATE") == 0))
  {
    return BLUETOOTH_CMD_MPU_STATE;
  }

  if ((strcmp(line, "RESET FRONT") == 0) ||
      (strcmp(line, "FRONT RESET") == 0) ||
      (strcmp(line, "DIR RESET") == 0) ||
      (strcmp(line, "RESET DIR") == 0) ||
      (strcmp(line, "HEADING RESET") == 0) ||
      (strcmp(line, "RESET HEADING") == 0) ||
      (strcmp(line, "MPU RESET FRONT") == 0) ||
      (strcmp(line, "MAP NORTH RESET") == 0) ||
      (strcmp(line, "RESET NORTH") == 0))
  {
    return BLUETOOTH_CMD_DIR_RESET;
  }

  if ((strcmp(line, "FRONT") == 0) ||
      (strcmp(line, "FRONT STATE") == 0) ||
      (strcmp(line, "FRONT STATUS") == 0) ||
      (strcmp(line, "LIDAR FRONT") == 0) ||
      (strcmp(line, "LIDAR FRONT STATE") == 0) ||
      (strcmp(line, "LIDAR FRONT STATUS") == 0) ||
      (strcmp(line, "LIDAR FRONT AREA") == 0) ||
      (strcmp(line, "LIDAR FRONT AREA STA") == 0) ||
      (strcmp(line, "LIDAR FRONT AREA STATUS") == 0))
  {
    return BLUETOOTH_CMD_LIDAR_FRONT_STATE;
  }

  if ((strcmp(line, "DIR") == 0) ||
      (strcmp(line, "DIRECTION") == 0) ||
      (strcmp(line, "HEADING") == 0) ||
      (strcmp(line, "HEADING STATE") == 0))
  {
    return BLUETOOTH_CMD_DIR_STATE;
  }

  if (BluetoothControl_IsTurnDegreeCommand(line, 'L') ||
      BluetoothControl_IsTurnDegreeCommand(line, 'A'))
  {
    return BLUETOOTH_CMD_TURN_LEFT_DEG;
  }

  if (BluetoothControl_IsTurnDegreeCommand(line, 'R') ||
      BluetoothControl_IsTurnDegreeCommand(line, 'D'))
  {
    return BLUETOOTH_CMD_TURN_RIGHT_DEG;
  }

  if (strcmp(line, "GO") == 0)
  {
    return BLUETOOTH_CMD_SLAM_NAV_ON;
  }

  if ((strcmp(line, "FWD") == 0) ||
      (strcmp(line, "FORWARD") == 0))
  {
    return BLUETOOTH_CMD_DRIVE_FORWARD;
  }

  if ((strcmp(line, "LEFT") == 0) ||
      (strcmp(line, "TURN LEFT") == 0))
  {
    return BLUETOOTH_CMD_TURN_LEFT;
  }

  if ((strcmp(line, "RIGHT") == 0) ||
      (strcmp(line, "TURN RIGHT") == 0))
  {
    return BLUETOOTH_CMD_TURN_RIGHT;
  }

  if ((strcmp(line, "DRIVE STOP") == 0) ||
      (strcmp(line, "MOTOR STOP") == 0) ||
      (strcmp(line, "BRAKE") == 0))
  {
    return BLUETOOTH_CMD_DRIVE_STOP;
  }

  return BLUETOOTH_CMD_UNKNOWN;
}

static void BluetoothControl_NormalizeLine(const char *input, char *output, size_t output_size)
{
  size_t in_index;
  size_t out_index = 0U;
  bool previous_space = true;

  if (output_size == 0U)
  {
    return;
  }

  for (in_index = 0U; (input[in_index] != '\0') && (out_index < (output_size - 1U)); ++in_index)
  {
    char c = input[in_index];

    if ((c == '\t') || (c == '-') || (c == '_'))
    {
      c = ' ';
    }

    if (c == ' ')
    {
      if (!previous_space)
      {
        output[out_index++] = ' ';
      }
      previous_space = true;
      continue;
    }

    output[out_index++] = BluetoothControl_ToUpper(c);
    previous_space = false;
  }

  if ((out_index > 0U) && (output[out_index - 1U] == ' '))
  {
    out_index--;
  }
  output[out_index] = '\0';
}

static void BluetoothControl_ApplyCommand(BluetoothCommandType_t command)
{
  s_state.last_command = command;

  if ((command == BLUETOOTH_CMD_START_MAPPING) ||
      (command == BLUETOOTH_CMD_MODE86_ON) ||
      (command == BLUETOOTH_CMD_SLAM_NAV_ON) ||
      (command == BLUETOOTH_CMD_SLAM_NAV_RETURN))
  {
    s_state.mapping_active = true;
  }
  else if ((command == BLUETOOTH_CMD_STOP_MAPPING) ||
           (command == BLUETOOTH_CMD_STOP_ALL) ||
           (command == BLUETOOTH_CMD_MODE86_OFF) ||
           (command == BLUETOOTH_CMD_SLAM_NAV_OFF))
  {
    s_state.mapping_active = false;
  }
  else if (command == BLUETOOTH_CMD_DEBUG_ON)
  {
    s_state.debug_enabled = true;
  }
  else if (command == BLUETOOTH_CMD_DEBUG_OFF)
  {
    s_state.debug_enabled = false;
  }
}

static void BluetoothControl_QueueCommand(BluetoothCommandType_t command, const char *text, uint32_t tick_ms)
{
  BluetoothCommand_t item;

  if (s_command_queue == NULL)
  {
    return;
  }

  memset(&item, 0, sizeof(item));
  item.type = command;
  item.tick_count = tick_ms;
  strncpy(item.text, text, sizeof(item.text) - 1U);

  if (BluetoothControl_IsUrgentCommand(command))
  {
    if (xQueueSendToFront(s_command_queue, &item, 0U) == pdPASS)
    {
      return;
    }

    {
      BluetoothCommand_t dropped_item;

      (void)xQueueReceive(s_command_queue, &dropped_item, 0U);
    }
    if (xQueueSendToFront(s_command_queue, &item, 0U) == pdPASS)
    {
      s_state.command_drops++;
      return;
    }
  }

  if (xQueueSend(s_command_queue, &item, 0U) != pdPASS)
  {
    BluetoothCommand_t dropped_item;

    (void)xQueueReceive(s_command_queue, &dropped_item, 0U);
    if (xQueueSend(s_command_queue, &item, 0U) == pdPASS)
    {
      s_state.command_drops++;
      return;
    }

    s_state.command_drops++;
  }
}

static void BluetoothControl_SendAck(BluetoothCommandType_t command)
{
  char response[48];

  (void)snprintf(response, sizeof(response), "ACK %s\r\n", BluetoothControl_CommandName(command));
  (void)BluetoothControl_SendText(response);
}

static void BluetoothControl_QueueLineFromIsr(const char *text, BaseType_t *higher_priority_task_woken)
{
  BluetoothLine_t completed_line;
  BluetoothLine_t dropped_line;

  if ((text == NULL) || (s_rx_line_queue == NULL))
  {
    return;
  }

  completed_line.tick_ms = HAL_GetTick();
  memset(completed_line.text, 0, sizeof(completed_line.text));
  strncpy(completed_line.text, text, sizeof(completed_line.text) - 1U);

  if (xQueueSendFromISR(s_rx_line_queue, &completed_line, higher_priority_task_woken) != pdPASS)
  {
    (void)xQueueReceiveFromISR(s_rx_line_queue, &dropped_line, higher_priority_task_woken);
    if (xQueueSendFromISR(s_rx_line_queue, &completed_line, higher_priority_task_woken) == pdPASS)
    {
      s_state.command_drops++;
      return;
    }

    s_state.command_drops++;
  }
}

static bool BluetoothControl_IsLineBreak(uint8_t byte)
{
  return ((byte == '\r') || (byte == '\n'));
}

static bool BluetoothControl_IsPrintable(uint8_t byte)
{
  return ((byte >= 0x20U) && (byte <= 0x7EU));
}

static bool BluetoothControl_IsUnsignedNumber(const char *line)
{
  if ((line == NULL) || (line[0] == '\0'))
  {
    return false;
  }

  while (*line != '\0')
  {
    if ((*line < '0') || (*line > '9'))
    {
      return false;
    }
    line++;
  }

  return true;
}

static bool BluetoothControl_IsSafeValueLine(const char *line)
{
  const char *text = line;

  if (line == NULL)
  {
    return false;
  }

  if (strncmp(text, "SAFE ", 5U) == 0)
  {
    text = &text[5];
  }

  if (strncmp(text, "VALUE ", 6U) == 0)
  {
    text = &text[6];
  }

  while (*text == ' ')
  {
    text++;
  }

  return BluetoothControl_IsUnsignedNumber(text);
}

static bool BluetoothControl_IsTurnDegreeCommand(const char *line, char prefix)
{
  const char *text = line;
  bool has_digit = false;

  if ((line == NULL) || (line[0] == '\0'))
  {
    return false;
  }

  if ((prefix == 'L') && (strncmp(line, "LEFT", 4U) == 0))
  {
    text = &line[4];
  }
  else if ((prefix == 'R') && (strncmp(line, "RIGHT", 5U) == 0))
  {
    text = &line[5];
  }
  else if (line[0] == prefix)
  {
    text = &line[1];
  }
  else
  {
    return false;
  }

  while (*text == ' ')
  {
    text++;
  }

  while (*text != '\0')
  {
    if ((*text < '0') || (*text > '9'))
    {
      return false;
    }
    has_digit = true;
    text++;
  }

  return has_digit;
}

static char BluetoothControl_ToUpper(char c)
{
  if ((c >= 'a') && (c <= 'z'))
  {
    return (char)(c - ('a' - 'A'));
  }

  return c;
}
