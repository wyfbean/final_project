#include "lidar_pipeline.h"

#include <string.h>

#include "bluetooth_control.h"
#include "FreeRTOS.h"
#include "main.h"
#include "queue.h"
#include "task.h"

extern UART_HandleTypeDef huart1;

#define RPLIDAR_CMD_STOP              0x25U
#define RPLIDAR_CMD_SCAN              0x20U
#define RPLIDAR_RESPONSE_TYPE_SCAN    0x81U
#define RPLIDAR_SCAN_NODE_PAYLOAD_LEN 5UL
#define LIDAR_DMA_BUFFER_SIZE         1024U
#define LIDAR_DMA_BLOCK_SIZE          (LIDAR_DMA_BUFFER_SIZE / 2U)
#define LIDAR_RESPONSE_DESCRIPTOR_LEN 7U
#define LIDAR_NODE_SIZE               5U
#define LIDAR_BLOCK_QUEUE_LENGTH      8U
#define LIDAR_RESULT_QUEUE_LENGTH     8U
#define LIDAR_POINT_QUEUE_LENGTH      512U
#define LIDAR_PARSER_TASK_STACK_WORDS 384U
#define LIDAR_SERVICE_TASK_STACK_WORDS 384U
#define LIDAR_PARSER_TASK_PRIORITY    (tskIDLE_PRIORITY + 3U)
#define LIDAR_SERVICE_TASK_PRIORITY   (tskIDLE_PRIORITY + 2U)
#define LIDAR_PROTOCOL_Q6_TO_CDEG_NUMERATOR   100UL
#define LIDAR_PROTOCOL_Q6_TO_CDEG_DENOMINATOR 64UL
#define LIDAR_BODY_ANGLE_SIGN                 (-1L)
#define LIDAR_BODY_YAW_OFFSET_CDEG            18000L
#define LIDAR_DEFAULT_MIN_POINT_QUALITY       5U
#define LIDAR_MAX_POINT_QUALITY               63U
#define LIDAR_DISTANCE_BIAS_MM                350U

typedef struct
{
  uint16_t angle_cdeg;
  uint16_t distance_mm;
  uint8_t quality;
  uint8_t is_scan_start;
} LidarNode_t;

static uint8_t s_lidar_dma_buffer[LIDAR_DMA_BUFFER_SIZE];

static StaticQueue_t s_block_queue_struct;
static uint8_t s_block_queue_storage[LIDAR_BLOCK_QUEUE_LENGTH * sizeof(LidarDmaBlockDescriptor_t)];
static QueueHandle_t s_block_queue;

static StaticQueue_t s_result_queue_struct;
static uint8_t s_result_queue_storage[LIDAR_RESULT_QUEUE_LENGTH * sizeof(LidarParseResult_t)];
static QueueHandle_t s_result_queue;

static StaticQueue_t s_point_queue_struct;
static uint8_t s_point_queue_storage[LIDAR_POINT_QUEUE_LENGTH * sizeof(LidarPoint_t)];
static QueueHandle_t s_point_queue;

static StaticTask_t s_parser_task_struct;
static StackType_t s_parser_task_stack[LIDAR_PARSER_TASK_STACK_WORDS];
static TaskHandle_t s_parser_task_handle;

static StaticTask_t s_service_task_struct;
static StackType_t s_service_task_stack[LIDAR_SERVICE_TASK_STACK_WORDS];
static TaskHandle_t s_service_task_handle;

static uint8_t s_descriptor_buffer[LIDAR_RESPONSE_DESCRIPTOR_LEN];
static uint8_t s_node_buffer[LIDAR_NODE_SIZE];
static uint8_t s_descriptor_fill;
static uint8_t s_descriptor_seen;
static uint8_t s_node_fill;

static volatile uint32_t s_block_sequence;
static volatile uint32_t s_dma_queue_drop_count;
static volatile uint32_t s_result_queue_drop_count;
static volatile uint32_t s_point_queue_drop_count;
static volatile uint32_t s_total_bytes;
static volatile uint32_t s_total_node_count;
static volatile uint32_t s_total_scan_start_count;
static volatile uint32_t s_uart_error_count;
static volatile uint8_t s_restart_pending;
static uint8_t s_min_point_quality = LIDAR_DEFAULT_MIN_POINT_QUALITY;
static bool s_scan_command_sent;
static bool s_pipeline_initialized;
static bool s_latest_result_valid;
static LidarParseResult_t s_latest_result;

static void LidarParserTask(void *argument);
static void LidarServiceTask(void *argument);
static void LidarPipeline_OnDmaBlockReadyFromIsr(LidarDmaEvent_t event, uint16_t offset);
static bool Lidar_StartReception(void);
static void LidarParser_ResetResult(LidarParseResult_t *result, const LidarDmaBlockDescriptor_t *descriptor);
static void LidarParser_ProcessBlock(const LidarDmaBlockDescriptor_t *descriptor, LidarParseResult_t *result);
static void LidarParser_ProcessByte(uint8_t byte, LidarParseResult_t *result);
static bool LidarDescriptor_IsStandardScan(const uint8_t descriptor[LIDAR_RESPONSE_DESCRIPTOR_LEN]);
static bool LidarNode_TryDecode(const uint8_t raw_node[LIDAR_NODE_SIZE], LidarNode_t *node);
static void LidarPoint_Publish(const LidarNode_t *node);
static bool LidarCommand_SendRequest(uint8_t command);
static bool LidarCommand_StartScan(void);
static uint16_t LidarProtocol_AngleQ6ToCdeg(uint16_t angle_q6);
static uint16_t LidarPipeline_CorrectDistanceMm(uint16_t raw_distance_mm);
static int32_t LidarPipeline_NormalizeAngleCdeg(int32_t angle_cdeg);

bool LidarPipeline_Init(void)
{
  if (s_pipeline_initialized)
  {
    return true;
  }

  s_block_queue = xQueueCreateStatic(
      LIDAR_BLOCK_QUEUE_LENGTH,
      sizeof(LidarDmaBlockDescriptor_t),
      s_block_queue_storage,
      &s_block_queue_struct);
  configASSERT(s_block_queue != NULL);

  s_result_queue = xQueueCreateStatic(
      LIDAR_RESULT_QUEUE_LENGTH,
      sizeof(LidarParseResult_t),
      s_result_queue_storage,
      &s_result_queue_struct);
  configASSERT(s_result_queue != NULL);

  s_point_queue = xQueueCreateStatic(
      LIDAR_POINT_QUEUE_LENGTH,
      sizeof(LidarPoint_t),
      s_point_queue_storage,
      &s_point_queue_struct);
  configASSERT(s_point_queue != NULL);

  s_parser_task_handle = xTaskCreateStatic(
      LidarParserTask,
      "lidarParse",
      LIDAR_PARSER_TASK_STACK_WORDS,
      NULL,
      LIDAR_PARSER_TASK_PRIORITY,
      s_parser_task_stack,
      &s_parser_task_struct);
  configASSERT(s_parser_task_handle != NULL);

  s_service_task_handle = xTaskCreateStatic(
      LidarServiceTask,
      "lidarSvc",
      LIDAR_SERVICE_TASK_STACK_WORDS,
      NULL,
      LIDAR_SERVICE_TASK_PRIORITY,
      s_service_task_stack,
      &s_service_task_struct);
  configASSERT(s_service_task_handle != NULL);

  if (!Lidar_StartReception())
  {
    return false;
  }

  s_pipeline_initialized = true;
  return true;
}

bool LidarPipeline_GetLatestResult(LidarParseResult_t *out_result)
{
  bool is_valid;

  if (out_result == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  is_valid = s_latest_result_valid;
  if (is_valid)
  {
    *out_result = s_latest_result;
  }
  taskEXIT_CRITICAL();

  return is_valid;
}

bool LidarPipeline_TakePoint(LidarPoint_t *out_point)
{
  if ((out_point == NULL) || (s_point_queue == NULL))
  {
    return false;
  }

  return (xQueueReceive(s_point_queue, out_point, 0U) == pdPASS);
}

uint32_t LidarPipeline_GetPointQueueDrops(void)
{
  uint32_t drops;

  taskENTER_CRITICAL();
  drops = s_point_queue_drop_count;
  taskEXIT_CRITICAL();

  return drops;
}

void LidarPipeline_SetMinPointQuality(uint8_t min_quality)
{
  if (min_quality > LIDAR_MAX_POINT_QUALITY)
  {
    min_quality = LIDAR_MAX_POINT_QUALITY;
  }

  taskENTER_CRITICAL();
  s_min_point_quality = min_quality;
  taskEXIT_CRITICAL();
}

uint8_t LidarPipeline_GetMinPointQuality(void)
{
  uint8_t min_quality;

  taskENTER_CRITICAL();
  min_quality = s_min_point_quality;
  taskEXIT_CRITICAL();
  return min_quality;
}

uint16_t LidarPipeline_GetDistanceBiasMm(void)
{
  return LIDAR_DISTANCE_BIAS_MM;
}

int32_t LidarPipeline_LidarToRobotAngleCdeg(uint16_t lidar_angle_cdeg)
{
  /*
   * RPLIDAR C1 protocol geometry:
   *   angle_q6 / 64 is the angle from the LiDAR forward X axis, increasing
   *   clockwise toward the LiDAR +Y axis. This vehicle's LiDAR is mounted
   *   reversed relative to the car front and mirrored in body coordinates, so
   *   the shared conversion is:
   *     body_angle = normalize(180deg - lidar_angle)
   */
  return LidarPipeline_NormalizeAngleCdeg(
      LIDAR_BODY_YAW_OFFSET_CDEG + (LIDAR_BODY_ANGLE_SIGN * (int32_t)lidar_angle_cdeg));
}

uint16_t LidarPipeline_LidarToRobotAngleU16(uint16_t lidar_angle_cdeg)
{
  return (uint16_t)LidarPipeline_LidarToRobotAngleCdeg(lidar_angle_cdeg);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    LidarPipeline_OnDmaBlockReadyFromIsr(LIDAR_DMA_EVENT_HALF, 0U);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    LidarPipeline_OnDmaBlockReadyFromIsr(LIDAR_DMA_EVENT_FULL, LIDAR_DMA_BLOCK_SIZE);
  }
  else if (huart->Instance == USART6)
  {
    BluetoothControl_OnUartRxCpltFromIsr(huart);
  }
  else if (huart->Instance == USART2)
  {
    BluetoothControl_OnUartRxCpltFromIsr(huart);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    s_uart_error_count++;
    s_restart_pending = 1U;
  }
  else if (huart->Instance == USART6)
  {
    BluetoothControl_OnUartErrorFromIsr(huart);
  }
  else if (huart->Instance == USART2)
  {
    BluetoothControl_OnUartErrorFromIsr(huart);
  }
}

static void LidarPipeline_OnDmaBlockReadyFromIsr(LidarDmaEvent_t event, uint16_t offset)
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  LidarDmaBlockDescriptor_t descriptor;

  if (s_block_queue == NULL)
  {
    return;
  }

  descriptor.offset = offset;
  descriptor.length = LIDAR_DMA_BLOCK_SIZE;
  descriptor.block_id = ++s_block_sequence;
  descriptor.tick_count = xTaskGetTickCountFromISR();
  descriptor.event = event;

  if (xQueueSendFromISR(s_block_queue, &descriptor, &higher_priority_task_woken) != pdPASS)
  {
    s_dma_queue_drop_count++;
  }

  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static bool Lidar_StartReception(void)
{
  s_descriptor_fill = 0U;
  s_descriptor_seen = 0U;
  s_node_fill = 0U;
  memset(s_lidar_dma_buffer, 0, sizeof(s_lidar_dma_buffer));

  (void)HAL_UART_DMAStop(&huart1);

  if (HAL_UART_Receive_DMA(&huart1, s_lidar_dma_buffer, sizeof(s_lidar_dma_buffer)) != HAL_OK)
  {
    return false;
  }

  return true;
}

static void LidarParserTask(void *argument)
{
  LidarDmaBlockDescriptor_t descriptor;
  LidarParseResult_t result;

  (void)argument;

  for (;;)
  {
    if (xQueueReceive(s_block_queue, &descriptor, portMAX_DELAY) != pdPASS)
    {
      continue;
    }

    LidarParser_ResetResult(&result, &descriptor);
    LidarParser_ProcessBlock(&descriptor, &result);

    if (xQueueSend(s_result_queue, &result, 0U) != pdPASS)
    {
      s_result_queue_drop_count++;
    }
  }
}

static void LidarServiceTask(void *argument)
{
  LidarParseResult_t result;

  (void)argument;

  vTaskDelay(pdMS_TO_TICKS(20U));

  for (;;)
  {
    if (!s_scan_command_sent)
    {
      if (LidarCommand_StartScan())
      {
        s_scan_command_sent = true;
      }
    }

    if (s_restart_pending != 0U)
    {
      s_restart_pending = 0U;
      if (Lidar_StartReception())
      {
        s_scan_command_sent = false;
        vTaskDelay(pdMS_TO_TICKS(5U));
      }
    }

    if (xQueueReceive(s_result_queue, &result, pdMS_TO_TICKS(100U)) == pdPASS)
    {
      taskENTER_CRITICAL();
      s_latest_result = result;
      s_latest_result_valid = true;
      taskEXIT_CRITICAL();
    }
  }
}

static void LidarParser_ResetResult(LidarParseResult_t *result, const LidarDmaBlockDescriptor_t *descriptor)
{
  memset(result, 0, sizeof(*result));
  result->block_id = descriptor->block_id;
  result->tick_count = descriptor->tick_count;
  result->event = descriptor->event;
  result->block_bytes = descriptor->length;
  result->min_distance_mm = UINT16_MAX;
}

static void LidarParser_ProcessBlock(const LidarDmaBlockDescriptor_t *descriptor, LidarParseResult_t *result)
{
  size_t index;

  s_total_bytes += descriptor->length;
  result->total_bytes = s_total_bytes;

  for (index = 0U; index < descriptor->length; ++index)
  {
    LidarParser_ProcessByte(s_lidar_dma_buffer[descriptor->offset + index], result);
  }

  if (result->min_distance_mm == UINT16_MAX)
  {
    result->min_distance_mm = 0U;
  }

  result->descriptor_seen = s_descriptor_seen;
  result->parser_pending_bytes = (uint8_t)(s_descriptor_seen ? s_node_fill : s_descriptor_fill);
  result->uart_error_count = s_uart_error_count;
  result->dma_queue_drops = s_dma_queue_drop_count;
  result->result_queue_drops = s_result_queue_drop_count;
  result->point_queue_drops = s_point_queue_drop_count;
}

static void LidarParser_ProcessByte(uint8_t byte, LidarParseResult_t *result)
{
  LidarNode_t node;

  if (s_descriptor_seen == 0U)
  {
    s_descriptor_buffer[s_descriptor_fill++] = byte;

    if (s_descriptor_fill >= LIDAR_RESPONSE_DESCRIPTOR_LEN)
    {
      if (LidarDescriptor_IsStandardScan(s_descriptor_buffer))
      {
        s_descriptor_seen = 1U;
        s_descriptor_fill = 0U;
      }
      else
      {
        memmove(&s_descriptor_buffer[0], &s_descriptor_buffer[1], LIDAR_RESPONSE_DESCRIPTOR_LEN - 1U);
        s_descriptor_fill = LIDAR_RESPONSE_DESCRIPTOR_LEN - 1U;
        result->invalid_nodes++;
      }
    }

    return;
  }

  s_node_buffer[s_node_fill++] = byte;
  if (s_node_fill < LIDAR_NODE_SIZE)
  {
    return;
  }

  if (LidarNode_TryDecode(s_node_buffer, &node))
  {
    result->valid_nodes++;
    result->last_angle_cdeg = node.angle_cdeg;
    result->last_distance_mm = node.distance_mm;
    result->last_quality = node.quality;

    if (result->valid_nodes == 1U)
    {
      result->first_angle_cdeg = node.angle_cdeg;
    }

    if ((node.distance_mm > 0U) && (node.distance_mm < result->min_distance_mm))
    {
      result->min_distance_mm = node.distance_mm;
    }

    if (node.is_scan_start != 0U)
    {
      result->scan_start_nodes++;
      s_total_scan_start_count++;
    }

    LidarPoint_Publish(&node);
    s_total_node_count++;
    s_node_fill = 0U;
  }
  else
  {
    memmove(&s_node_buffer[0], &s_node_buffer[1], LIDAR_NODE_SIZE - 1U);
    s_node_fill = LIDAR_NODE_SIZE - 1U;
    result->invalid_nodes++;
  }
}

static bool LidarDescriptor_IsStandardScan(const uint8_t descriptor[LIDAR_RESPONSE_DESCRIPTOR_LEN])
{
  uint32_t response_length;

  if ((descriptor[0] != 0xA5U) || (descriptor[1] != 0x5AU))
  {
    return false;
  }

  response_length = ((uint32_t)descriptor[2]) |
                    ((uint32_t)descriptor[3] << 8U) |
                    ((uint32_t)descriptor[4] << 16U) |
                    (((uint32_t)descriptor[5] & 0x3FUL) << 24U);

  return ((response_length == RPLIDAR_SCAN_NODE_PAYLOAD_LEN) &&
          (descriptor[6] == RPLIDAR_RESPONSE_TYPE_SCAN));
}

static bool LidarNode_TryDecode(const uint8_t raw_node[LIDAR_NODE_SIZE], LidarNode_t *node)
{
  const uint8_t start_bit = raw_node[0] & 0x01U;
  const uint8_t not_start_bit = (raw_node[0] >> 1U) & 0x01U;
  const uint16_t angle_q6 = (uint16_t)((((uint16_t)raw_node[2] << 8) | raw_node[1]) >> 1U);
  const uint16_t distance_q2 = (uint16_t)(((uint16_t)raw_node[4] << 8) | raw_node[3]);

  if (start_bit == not_start_bit)
  {
    return false;
  }

  if ((raw_node[1] & 0x01U) == 0U)
  {
    return false;
  }

  node->is_scan_start = start_bit;
  node->quality = raw_node[0] >> 2U;
  node->angle_cdeg = LidarProtocol_AngleQ6ToCdeg(angle_q6);
  node->distance_mm = LidarPipeline_CorrectDistanceMm((uint16_t)(distance_q2 / 4U));

  return true;
}

static uint16_t LidarProtocol_AngleQ6ToCdeg(uint16_t angle_q6)
{
  uint32_t angle_cdeg = (((uint32_t)angle_q6 * LIDAR_PROTOCOL_Q6_TO_CDEG_NUMERATOR) +
                         (LIDAR_PROTOCOL_Q6_TO_CDEG_DENOMINATOR / 2UL)) /
                        LIDAR_PROTOCOL_Q6_TO_CDEG_DENOMINATOR;

  return (uint16_t)(angle_cdeg % 36000UL);
}

static uint16_t LidarPipeline_CorrectDistanceMm(uint16_t raw_distance_mm)
{
  if (raw_distance_mm == 0U)
  {
    return 0U;
  }

  if (raw_distance_mm <= LIDAR_DISTANCE_BIAS_MM)
  {
    return 1U;
  }

  return (uint16_t)(raw_distance_mm - LIDAR_DISTANCE_BIAS_MM);
}

static void LidarPoint_Publish(const LidarNode_t *node)
{
  LidarPoint_t point;
  LidarPoint_t dropped_point;

  if ((node == NULL) || (s_point_queue == NULL))
  {
    return;
  }

  if (node->quality < LidarPipeline_GetMinPointQuality())
  {
    return;
  }

  point.tick_count = xTaskGetTickCount();
  point.angle_cdeg = node->angle_cdeg;
  point.distance_mm = node->distance_mm;
  point.quality = node->quality;
  point.flags = (node->is_scan_start != 0U) ? LIDAR_POINT_FLAG_SCAN_START : 0U;

  if (xQueueSend(s_point_queue, &point, 0U) == pdPASS)
  {
    return;
  }

  s_point_queue_drop_count++;
  if ((xQueueReceive(s_point_queue, &dropped_point, 0U) == pdPASS) &&
      (xQueueSend(s_point_queue, &point, 0U) == pdPASS))
  {
    return;
  }
}

static bool LidarCommand_SendRequest(uint8_t command)
{
  const uint8_t packet[2] = {0xA5U, command};

  return HAL_UART_Transmit(&huart1, (uint8_t *)packet, sizeof(packet), 100U) == HAL_OK;
}

static bool LidarCommand_StartScan(void)
{
  if (!LidarCommand_SendRequest(RPLIDAR_CMD_STOP))
  {
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(5U));

  if (!LidarCommand_SendRequest(RPLIDAR_CMD_SCAN))
  {
    return false;
  }

  return true;
}

static int32_t LidarPipeline_NormalizeAngleCdeg(int32_t angle_cdeg)
{
  while (angle_cdeg < 0L)
  {
    angle_cdeg += 36000L;
  }

  while (angle_cdeg >= 36000L)
  {
    angle_cdeg -= 36000L;
  }

  return angle_cdeg;
}
