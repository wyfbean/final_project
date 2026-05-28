#ifndef BLUETOOTH_CONTROL_H
#define BLUETOOTH_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#define BLUETOOTH_LINE_MAX 32U

typedef enum
{
  BLUETOOTH_CMD_NONE = 0,
  BLUETOOTH_CMD_START_MAPPING,
  BLUETOOTH_CMD_STOP_MAPPING,
  BLUETOOTH_CMD_STOP_ALL,
  BLUETOOTH_CMD_SHOW_MAP_RESULT,
  BLUETOOTH_CMD_DEBUG_ON,
  BLUETOOTH_CMD_DEBUG_OFF,
  BLUETOOTH_CMD_LIDAR_DEBUG_ON,
  BLUETOOTH_CMD_LIDAR_DEBUG_OFF,
  BLUETOOTH_CMD_LIDAR_QUALITY_SET,
  BLUETOOTH_CMD_SAFE_SET,
  BLUETOOTH_CMD_SAFE_VALUE,
  BLUETOOTH_CMD_SAFE_END,
  BLUETOOTH_CMD_SAFE_UP,
  BLUETOOTH_CMD_SAFE_DOWN,
  BLUETOOTH_CMD_ODOM_DEBUG_ON,
  BLUETOOTH_CMD_ODOM_DEBUG_OFF,
  BLUETOOTH_CMD_ENCODER_CAL_END,
  BLUETOOTH_CMD_ENCODER_SCALE_SET,
  BLUETOOTH_CMD_ENCODER_DEBUG,
  BLUETOOTH_CMD_AUTO_MAPPING_ON,
  BLUETOOTH_CMD_AUTO_MAPPING_OFF,
  BLUETOOTH_CMD_MODE86_ON,
  BLUETOOTH_CMD_MODE86_OFF,
  BLUETOOTH_CMD_SLAM_NAV_ON,
  BLUETOOTH_CMD_SLAM_NAV_OFF,
  BLUETOOTH_CMD_SLAM_NAV_RETURN,
  BLUETOOTH_CMD_GYRO_CALIBRATE,
  BLUETOOTH_CMD_MPU_STATE,
  BLUETOOTH_CMD_LIDAR_FRONT_STATE,
  BLUETOOTH_CMD_DIR_RESET,
  BLUETOOTH_CMD_DIR_STATE,
  BLUETOOTH_CMD_TURN_LEFT_DEG,
  BLUETOOTH_CMD_TURN_RIGHT_DEG,
  BLUETOOTH_CMD_DRIVE_FORWARD,
  BLUETOOTH_CMD_TURN_LEFT,
  BLUETOOTH_CMD_TURN_RIGHT,
  BLUETOOTH_CMD_DRIVE_STOP,
  BLUETOOTH_CMD_GYRO_SCALE_SET,
  BLUETOOTH_CMD_UNKNOWN
} BluetoothCommandType_t;

typedef struct
{
  BluetoothCommandType_t type;
  uint32_t tick_count;
  char text[BLUETOOTH_LINE_MAX];
} BluetoothCommand_t;

typedef struct
{
  bool ready;
  bool debug_enabled;
  bool mapping_active;
  BluetoothCommandType_t last_command;
  uint32_t rx_bytes;
  uint32_t rx_lines;
  uint32_t rx_overflows;
  uint32_t parse_errors;
  uint32_t command_drops;
  uint32_t uart_errors;
  uint32_t tx_count;
  uint32_t tx_drops;
  uint32_t last_rx_tick_ms;
  uint32_t last_tx_tick_ms;
  char last_line[BLUETOOTH_LINE_MAX];
} BluetoothControlState_t;

bool BluetoothControl_Init(void);
void BluetoothControl_Update(void);
bool BluetoothControl_TakeCommand(BluetoothCommand_t *out_command);
bool BluetoothControl_GetState(BluetoothControlState_t *out_state);
bool BluetoothControl_SendText(const char *text);
const char *BluetoothControl_CommandName(BluetoothCommandType_t command);

void BluetoothControl_OnUartRxCpltFromIsr(UART_HandleTypeDef *huart);
void BluetoothControl_OnUartErrorFromIsr(UART_HandleTypeDef *huart);

#endif /* BLUETOOTH_CONTROL_H */
