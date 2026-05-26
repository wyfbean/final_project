/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bluetooth_control.h"
#include "lidar_pipeline.h"
#include "mapping_grid.h"
#include "motor_control.h"
#include "mpu6500.h"
#include "slam_nav.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OLED_I2C_ADDR               (0x3CU << 1)
#define OLED_WIDTH                  128U
#define OLED_HEIGHT                 64U
#define OLED_PAGE_COUNT             (OLED_HEIGHT / 8U)
#define OLED_FB_SIZE                (OLED_WIDTH * OLED_PAGE_COUNT)
#define OLED_I2C_TIMEOUT_MS         20U
#define OLED_RECOVER_RETRY_MS       1000U

#define ADC_CHANNEL_COUNT           3U
#define BUTTON_DEBOUNCE_MS          30U
#define ADC_PWM_MIN_PERMILLE        0U
#define ADC_PWM_MAX_PERMILLE        1000U
#define ADC_PWM_UPDATE_DEADBAND     8U

#define MAPPING_POINT_BATCH_LIMIT   64U
#define MAP_ROW_TX_INTERVAL_MS      30U
#define MAP_ROWS_PER_TX_BURST       2U
#define MAP_STAT_TX_INTERVAL_MS     2000U
#define POSE_TX_INTERVAL_MS         100U
#define LIDAR_DEBUG_MAX_TX_PER_BATCH 4U
#define LIDAR_FRONT_STATE_MAX_AGE_MS 700U
#define ODOM_DEBUG_TX_INTERVAL_MS   500U
#define AUTO_MAPPING_CONTROL_INTERVAL_MS 50U
#define AUTO_MAPPING_FRONT_SECTOR_CDEG 1800U
#define AUTO_MAPPING_SIDE_SECTOR_CDEG  3000U
#define AUTO_MAPPING_RIGHT_CENTER_CDEG 9000U
#define AUTO_MAPPING_BACK_CENTER_CDEG 18000U
#define AUTO_MAPPING_LEFT_CENTER_CDEG  27000U
#define AUTO_MAPPING_FRONT_RIGHT_CENTER_CDEG 4500U
#define AUTO_MAPPING_FRONT_LEFT_CENTER_CDEG 31500U
#define AUTO_MAPPING_DIAGONAL_SECTOR_CDEG 1800U
#define AUTO_MAPPING_FRONT_BODY_OFFSET_MM 200U
#define AUTO_MAPPING_FRONT_DIAGONAL_BODY_OFFSET_MM 140U
#define AUTO_MAPPING_MIN_SAFE_MM    350U
#define AUTO_MAPPING_MAX_SAFE_MM    3000U
#define AUTO_MAPPING_FRONT_BLOCK_MAX_MM 900U
#define AUTO_MAPPING_MAX_DRIVE_PWM  1000U
#define AUTO_MAPPING_MAX_TURN_PWM   1000U
#define AUTO_MAPPING_OPEN_MARGIN_MM 180U
#define AUTO_MAPPING_RIGHT_OPEN_MARGIN_MM 450U
#define AUTO_MAPPING_MIN_RIGHT_OPEN_MM 750U
#define AUTO_MAPPING_MIN_FRONT_RIGHT_OPEN_MM 850U
#define AUTO_MAPPING_RIGHT_WALL_MARGIN_MM 260U
#define AUTO_MAPPING_RIGHT_BRANCH_CONFIRM_COUNT 5U
#define AUTO_MAPPING_OBSERVE_SCAN_STARTS 3U
#define AUTO_MAPPING_OBSERVE_TIMEOUT_MS 700U
#define AUTO_MAPPING_FRONT_BLOCK_HOLD_MS 400U
#define AUTO_MAPPING_GRID_TURN_DEG  90U
#define AUTO_MAPPING_U_TURN_DEG     180U
#define AUTO_MAPPING_MIN_TURN_DEG   40U
#define AUTO_MAPPING_MAX_TURN_DEG   180U
#define AUTO_MAPPING_TURN_SETTLE_MS 240U
#define AUTO_MAPPING_POST_TURN_DRIVE_MS 650U
#define ANGLE_TURN_DEFAULT_DEG      90U
#define ANGLE_TURN_MIN_DEG          1U
#define ANGLE_TURN_MAX_DEG          360U
#define ANGLE_TURN_DONE_TOL_CDEG    250L
#define ANGLE_TURN_CORRECTION_TOL_CDEG 300L
#define ANGLE_TURN_MAX_CORRECTIONS  2U
#define ANGLE_TURN_TIMEOUT_MS       12000U
#define ANGLE_TURN_SLOW_ZONE_CDEG   3000L
#define ANGLE_TURN_FINE_ZONE_CDEG   1200L
#define ANGLE_TURN_SLOW_PWM         450U
#define ANGLE_TURN_FINE_PWM         320U
#define MAPPING_START_X_MM          (-1500L)
#define MAPPING_START_Y_MM          (-1500L)
#define MAPPING_START_HEADING_CDEG  9000L
#define MAPPING_POSE_HISTORY_LENGTH 64U
#define MAPPING_LIDAR_POINT_MAX_AGE_MS 250U
#define ENCODER_RIGHT_DELTA_SIGN    (-1L)
#define MAPPING_ENCODER_MM_PER_COUNT_X1000 94L
#define ENCODER_CAL_DISTANCE_MM     350L
#define ENCODER_CAL_MIN_COUNTS      10L
#define GYRO_STATIONARY_COUNT_THRESHOLD 2L
#define GYRO_BIAS_STATIONARY_CONFIRM_SAMPLES 25U
#define GYRO_BIAS_FILTER_DIVISOR    16L
#define GYRO_DEADBAND_DPS_X100      25L
#define GYRO_CALIBRATION_DURATION_MS 2000U
#define GYRO_CALIBRATION_MIN_SAMPLES 50U
#define GYRO_CALIBRATION_MAX_MOVING_SAMPLES 8U
#define MAPPING_PI                  3.14159265358979323846f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId senseTaskHandle;
osThreadId uiTaskHandle;
osThreadId btTaskHandle;
osThreadId lidarAppTaskHandle;

typedef enum
{
  TEST_PAGE_LIDAR = 0,
  TEST_PAGE_MPU,
  TEST_PAGE_ADC,
  TEST_PAGE_ENCODER,
  TEST_PAGE_COUNT
} test_page_t;

typedef struct
{
  uint16_t last_left_counter;
  uint16_t last_right_counter;
  int16_t left_delta;
  int16_t right_delta;
  int32_t left_total;
  int32_t right_total;
} encoder_test_state_t;

typedef struct
{
  uint32_t scan_seq;
  uint32_t last_tick_ms;
  uint32_t min_tick_ms;
  uint16_t point_count;
  uint16_t min_distance_mm;
  uint16_t min_raw_angle_cdeg;
  uint16_t min_robot_angle_cdeg;
  uint8_t min_quality;
} lidar_front_stats_t;

static uint16_t adc_buf[ADC_CHANNEL_COUNT];
static uint8_t oled_fb[OLED_FB_SIZE];
static uint8_t oled_page_tx[OLED_WIDTH + 1U];

static volatile bool app_ready = false;
static bool oled_ready = false;
static test_page_t current_page = TEST_PAGE_LIDAR;
static uint8_t button_stable_mask = 0U;
static uint8_t button_prev_nonzero = 0U;
static uint8_t button_sample_mask = 0U;
static uint32_t button_change_tick_ms = 0U;
static uint32_t last_lidar_result_tick_ms = 0U;
static uint32_t last_oled_tick_ms = 0U;
static uint32_t last_oled_recover_tick_ms = 0U;
static uint32_t last_sensor_tick_ms = 0U;
static uint32_t last_map_row_tx_tick_ms = 0U;
static uint32_t last_map_stat_tx_tick_ms = 0U;
static uint32_t last_pose_tx_tick_ms = 0U;
static uint32_t last_mapping_pose_tick_ms = 0U;
static uint32_t mapping_start_tick_ms = 0U;

static bool lidar_result_valid = false;
static bool mapping_active = false;
static bool lidar_debug_active = false;
static bool odom_debug_active = false;
static bool auto_mapping_active = false;
static LidarParseResult_t lidar_result = {0};
static Mpu6500State_t mpu_state = {0};
static encoder_test_state_t encoder_test = {0};
static MappingGridPose_t mapping_pose = {0};
static uint8_t next_map_tx_row = 0U;
static uint32_t lidar_debug_seq = 0U;
static uint32_t lidar_front_scan_seq = 0U;
static lidar_front_stats_t lidar_front_current = {0};
static lidar_front_stats_t lidar_front_last = {0};
static int32_t mapping_travel_residual_x1000 = 0L;
static uint32_t last_odom_debug_update_tick_ms = 0U;
static uint32_t last_odom_debug_tx_tick_ms = 0U;
static uint32_t odom_debug_seq = 0U;
static int32_t odom_debug_left_counts = 0L;
static int32_t odom_debug_right_counts = 0L;
static int32_t odom_debug_heading_cdeg = 0L;
static int32_t mapping_encoder_mm_per_count_x1000 = MAPPING_ENCODER_MM_PER_COUNT_X1000;
static int32_t gyro_z_bias_dps_x100 = 0L;
static int32_t gyro_z_corrected_dps_x100 = 0L;
static uint8_t gyro_bias_stationary_samples = 0U;
static MappingGridPose_t mapping_pose_history[MAPPING_POSE_HISTORY_LENGTH];
static uint32_t mapping_pose_history_ticks[MAPPING_POSE_HISTORY_LENGTH];
static uint8_t mapping_pose_history_next = 0U;
static uint8_t mapping_pose_history_count = 0U;
static uint16_t auto_mapping_front_min_mm = UINT16_MAX;
static uint16_t auto_mapping_front_blocking_angle_cdeg = 0U;
static uint16_t auto_mapping_right_min_mm = UINT16_MAX;
static uint16_t auto_mapping_front_right_min_mm = UINT16_MAX;
static uint16_t auto_mapping_left_min_mm = UINT16_MAX;
static uint16_t auto_mapping_right_best_distance_mm = 0U;
static uint16_t auto_mapping_right_best_angle_cdeg = AUTO_MAPPING_RIGHT_CENTER_CDEG;
static uint16_t auto_mapping_left_best_distance_mm = 0U;
static uint16_t auto_mapping_left_best_angle_cdeg = AUTO_MAPPING_LEFT_CENTER_CDEG;
static uint16_t auto_mapping_escape_best_distance_mm = 0U;
static uint16_t auto_mapping_escape_best_angle_cdeg = AUTO_MAPPING_LEFT_CENTER_CDEG;
static uint16_t auto_mapping_last_turn_target_angle_cdeg = AUTO_MAPPING_LEFT_CENTER_CDEG;
static uint32_t last_auto_mapping_control_tick_ms = 0U;
static uint32_t auto_mapping_avoid_count = 0U;
static uint32_t auto_mapping_resume_tick_ms = 0U;
static uint32_t auto_mapping_ignore_right_until_ms = 0U;
static uint32_t auto_mapping_observe_deadline_ms = 0U;
static uint32_t auto_mapping_front_blocked_until_ms = 0U;
static uint32_t auto_mapping_front_last_seen_ms = 0U;
static uint16_t auto_mapping_front_last_robot_angle_cdeg = 0U;
static uint16_t auto_mapping_front_last_raw_angle_cdeg = 0U;
static uint8_t auto_mapping_right_branch_confirm_count = 0U;
static uint8_t auto_mapping_observe_scan_starts_remaining = 0U;
static bool auto_mapping_right_wall_seen = false;
static bool auto_mapping_observe_after_resume = false;
static bool gyro_calibration_active = false;
static uint32_t gyro_calibration_start_tick_ms = 0U;
static int64_t gyro_calibration_sum_dps_x100 = 0;
static uint16_t gyro_calibration_samples = 0U;
static uint16_t gyro_calibration_moving_samples = 0U;
static bool angle_turn_active = false;
static int8_t angle_turn_direction = 0;
static bool angle_turn_heading_target_valid = false;
static bool angle_turn_completed_by_progress = false;
static uint8_t angle_turn_correction_count = 0U;
static int32_t angle_turn_target_cdeg = 0L;
static int32_t angle_turn_target_heading_cdeg = 0L;
static int32_t angle_turn_progress_cdeg = 0L;
static uint32_t angle_turn_last_tick_ms = 0U;
static uint32_t angle_turn_start_tick_ms = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static void TestApp_InitPeripherals(void);
static void TestApp_PollButtons(void);
static void TestApp_UpdateSensors(void);
static void TestApp_UpdateDisplay(void);
static void TestApp_HandleBluetoothCommands(void);
static void TestApp_UpdateMotorSpeedFromAdc(void);
static void TestApp_StartMapping(void);
static void TestApp_ResumeMapping(void);
static void TestApp_StopMapping(void);
static void TestApp_ProcessLidarPoints(void);
static void TestApp_SendLidarDebugPoint(const LidarPoint_t *point);
static void TestApp_UpdateLidarFrontStats(const LidarPoint_t *point);
static void TestApp_ResetLidarFrontStats(lidar_front_stats_t *stats, uint32_t scan_seq);
static void TestApp_SendLidarFrontState(void);
static void TestApp_UpdateMappingPose(uint32_t now_ms, int16_t left_delta, int16_t right_delta);
static void TestApp_ResetPoseHistory(void);
static void TestApp_RecordPoseHistory(uint32_t tick_ms);
static bool TestApp_GetPoseForTick(uint32_t tick_ms, MappingGridPose_t *out_pose);
static void TestApp_StartOdomDebug(void);
static void TestApp_StopOdomDebug(void);
static void TestApp_EndEncoderCalibration(void);
static void TestApp_UpdateGyroDriftCompensation(int16_t left_delta, int16_t right_delta);
static void TestApp_StartGyroCalibration(void);
static void TestApp_UpdateGyroCalibration(uint32_t now_ms, int16_t left_delta, int16_t right_delta);
static void TestApp_UpdateOdomDebug(uint32_t now_ms, int16_t left_delta, int16_t right_delta);
static void TestApp_StreamOdomDebug(void);
static void TestApp_SetLidarQualityFilter(const char *text);
static void TestApp_StartAutoMapping(void);
static void TestApp_StopAutoMapping(void);
static void TestApp_UpdateAutoMapping(uint32_t now_ms);
static bool TestApp_IsFrontLidarPoint(uint16_t angle_cdeg);
static bool TestApp_IsLidarAngleNear(uint16_t angle_cdeg, uint16_t center_cdeg, uint16_t half_width_cdeg);
static void TestApp_UpdateAutoMappingObstacle(const LidarPoint_t *point);
static void TestApp_StartAutoObservation(const char *reason);
static void TestApp_ResetAutoMappingSectorMins(void);
static bool TestApp_IsAutoFrontBlocked(uint32_t now_ms);
static void TestApp_AutoMappingStartTurn(int8_t direction, uint16_t degrees, const char *reason, uint32_t now_ms);
static void TestApp_StartAngleTurn(int8_t direction, uint16_t degrees);
static void TestApp_StartHeadingTurn(int32_t target_heading_cdeg, uint8_t correction_count, const char *reason);
static void TestApp_StopAngleTurn(bool completed);
static void TestApp_UpdateAngleTurn(uint32_t now_ms);
static uint16_t TestApp_ParseTurnDegrees(const char *text);
static uint8_t TestApp_ParseLidarQuality(const char *text, uint8_t fallback);
static int32_t TestApp_SignedHeadingErrorCdeg(int32_t target_cdeg, int32_t current_cdeg);
static int32_t TestApp_SnapHeadingToMazeAxis(int32_t heading_cdeg);
static int32_t TestApp_GetAutoTurnTargetHeading(int8_t direction, uint16_t requested_degrees);
static void TestApp_StreamMap(void);
static void TestApp_StreamPose(void);
static void TestApp_RequestFullMapStream(void);
static void TestApp_SendMapHeader(const char *state);
static void TestApp_SendMapStat(void);
static void TestApp_SendMpuState(void);
static void TestApp_ResetMapNorth(void);
static void TestApp_SendDirState(void);
static int32_t AppAbs32(int32_t value);
static uint16_t ClampPwmPermille(uint16_t value, uint16_t max_value);
static uint16_t GetObstacleSafeDistanceMm(void);
static uint16_t GetAutoFrontBlockDistanceMm(void);
static uint16_t TestApp_ApplyForwardBodyOffset(uint16_t robot_angle_cdeg, uint16_t distance_mm);

static bool OLED_InitMinimal(void);
static bool OLED_WriteCommand(uint8_t command);
static bool OLED_WriteCommands(const uint8_t *commands, uint16_t length);
static void OLED_Clear(void);
static void OLED_Update(void);
static void OLED_DrawChar6x8(uint8_t x, uint8_t page, char c);
static void OLED_DrawString6x8(uint8_t x, uint8_t page, const char *text);

static int16_t Encoder_ComputeDelta(uint16_t current, uint16_t previous);
static uint8_t Buttons_ReadMask(void);
static void RenderPage_LiDAR(void);
static void RenderPage_MPU(void);
static void RenderPage_ADC(void);
static void RenderPage_Encoder(void);
static void DrawStatusLine(uint8_t page, const char *label, int32_t value);
static uint16_t AdcToPwmPermille(uint16_t adc_value);
static uint16_t GetDrivePwmPermille(void);
static uint16_t GetTurnPwmPermille(void);
static int32_t NormalizeHeadingCdeg(int32_t heading_cdeg);

void StartSenseTask(void const * argument);
void StartUiTask(void const * argument);
void StartBtTask(void const * argument);
void StartLidarAppTask(void const * argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const uint8_t font_space[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t font_dash[5]  = {0x08, 0x08, 0x08, 0x08, 0x08};
static const uint8_t font_dot[5]   = {0x00, 0x60, 0x60, 0x00, 0x00};
static const uint8_t font_colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
static const uint8_t font_slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
static const uint8_t font_equal[5] = {0x14, 0x14, 0x14, 0x14, 0x14};
static const uint8_t font_0[5] = {0x3E, 0x51, 0x49, 0x45, 0x3E};
static const uint8_t font_1[5] = {0x00, 0x42, 0x7F, 0x40, 0x00};
static const uint8_t font_2[5] = {0x42, 0x61, 0x51, 0x49, 0x46};
static const uint8_t font_3[5] = {0x21, 0x41, 0x45, 0x4B, 0x31};
static const uint8_t font_4[5] = {0x18, 0x14, 0x12, 0x7F, 0x10};
static const uint8_t font_5[5] = {0x27, 0x45, 0x45, 0x45, 0x39};
static const uint8_t font_6[5] = {0x3C, 0x4A, 0x49, 0x49, 0x30};
static const uint8_t font_7[5] = {0x01, 0x71, 0x09, 0x05, 0x03};
static const uint8_t font_8[5] = {0x36, 0x49, 0x49, 0x49, 0x36};
static const uint8_t font_9[5] = {0x06, 0x49, 0x49, 0x29, 0x1E};
static const uint8_t font_A[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
static const uint8_t font_B[5] = {0x7F, 0x49, 0x49, 0x49, 0x36};
static const uint8_t font_C[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
static const uint8_t font_D[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
static const uint8_t font_E[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
static const uint8_t font_F[5] = {0x7F, 0x09, 0x09, 0x09, 0x01};
static const uint8_t font_G[5] = {0x3E, 0x41, 0x49, 0x49, 0x7A};
static const uint8_t font_H[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
static const uint8_t font_I[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
static const uint8_t font_J[5] = {0x20, 0x40, 0x41, 0x3F, 0x01};
static const uint8_t font_K[5] = {0x7F, 0x08, 0x14, 0x22, 0x41};
static const uint8_t font_L[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
static const uint8_t font_M[5] = {0x7F, 0x02, 0x0C, 0x02, 0x7F};
static const uint8_t font_N[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F};
static const uint8_t font_O[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
static const uint8_t font_P[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
static const uint8_t font_Q[5] = {0x3E, 0x41, 0x51, 0x21, 0x5E};
static const uint8_t font_R[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
static const uint8_t font_S[5] = {0x46, 0x49, 0x49, 0x49, 0x31};
static const uint8_t font_T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
static const uint8_t font_U[5] = {0x3F, 0x40, 0x40, 0x40, 0x3F};
static const uint8_t font_V[5] = {0x1F, 0x20, 0x40, 0x20, 0x1F};
static const uint8_t font_W[5] = {0x7F, 0x20, 0x18, 0x20, 0x7F};
static const uint8_t font_X[5] = {0x63, 0x14, 0x08, 0x14, 0x63};
static const uint8_t font_Y[5] = {0x03, 0x04, 0x78, 0x04, 0x03};
static const uint8_t font_Z[5] = {0x61, 0x51, 0x49, 0x45, 0x43};

static const uint8_t *OLED_GetGlyph(char c)
{
  switch (c)
  {
    case ' ': return font_space;
    case '-': return font_dash;
    case '.': return font_dot;
    case ':': return font_colon;
    case '/': return font_slash;
    case '=': return font_equal;
    case '0': return font_0;
    case '1': return font_1;
    case '2': return font_2;
    case '3': return font_3;
    case '4': return font_4;
    case '5': return font_5;
    case '6': return font_6;
    case '7': return font_7;
    case '8': return font_8;
    case '9': return font_9;
    case 'A': return font_A;
    case 'B': return font_B;
    case 'C': return font_C;
    case 'D': return font_D;
    case 'E': return font_E;
    case 'F': return font_F;
    case 'G': return font_G;
    case 'H': return font_H;
    case 'I': return font_I;
    case 'J': return font_J;
    case 'K': return font_K;
    case 'L': return font_L;
    case 'M': return font_M;
    case 'N': return font_N;
    case 'O': return font_O;
    case 'P': return font_P;
    case 'Q': return font_Q;
    case 'R': return font_R;
    case 'S': return font_S;
    case 'T': return font_T;
    case 'U': return font_U;
    case 'V': return font_V;
    case 'W': return font_W;
    case 'X': return font_X;
    case 'Y': return font_Y;
    case 'Z': return font_Z;
    default:  return font_space;
  }
}

static bool OLED_WriteCommand(uint8_t command)
{
  uint8_t payload[2] = {0x00U, command};
  return (HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, payload, sizeof(payload), OLED_I2C_TIMEOUT_MS) == HAL_OK);
}

static bool OLED_WriteCommands(const uint8_t *commands, uint16_t length)
{
  uint16_t i;

  for (i = 0U; i < length; ++i)
  {
    if (!OLED_WriteCommand(commands[i]))
    {
      return false;
    }
  }

  return true;
}

static bool OLED_InitMinimal(void)
{
  static const uint8_t init_cmds[] =
  {
    0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
    0x8DU, 0x14U, 0x20U, 0x00U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
    0x81U, 0xCFU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U,
    0xAFU
  };

  if (HAL_I2C_IsDeviceReady(&hi2c2, OLED_I2C_ADDR, 3U, OLED_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  if (!OLED_WriteCommands(init_cmds, sizeof(init_cmds)))
  {
    return false;
  }

  OLED_Clear();
  OLED_Update();
  return true;
}

static void OLED_Clear(void)
{
  memset(oled_fb, 0, sizeof(oled_fb));
}

static void OLED_DrawChar6x8(uint8_t x, uint8_t page, char c)
{
  const uint8_t *glyph;
  uint16_t offset;
  uint8_t col;

  if ((x > (OLED_WIDTH - 6U)) || (page >= OLED_PAGE_COUNT))
  {
    return;
  }

  glyph = OLED_GetGlyph(c);
  offset = ((uint16_t)page * OLED_WIDTH) + x;

  for (col = 0U; col < 5U; ++col)
  {
    oled_fb[offset + col] = glyph[col];
  }
  oled_fb[offset + 5U] = 0x00U;
}

static void OLED_DrawString6x8(uint8_t x, uint8_t page, const char *text)
{
  while ((*text != '\0') && (x <= (OLED_WIDTH - 6U)))
  {
    OLED_DrawChar6x8(x, page, *text);
    x = (uint8_t)(x + 6U);
    ++text;
  }
}

static void OLED_Update(void)
{
  uint8_t page;
  uint8_t commands[3];

  if (!oled_ready)
  {
    return;
  }

  oled_page_tx[0] = 0x40U;

  for (page = 0U; page < OLED_PAGE_COUNT; ++page)
  {
    commands[0] = (uint8_t)(0xB0U + page);
    commands[1] = 0x00U;
    commands[2] = 0x10U;

    if (!OLED_WriteCommands(commands, sizeof(commands)))
    {
      oled_ready = false;
      return;
    }

    memcpy(&oled_page_tx[1], &oled_fb[page * OLED_WIDTH], OLED_WIDTH);
    if (HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, oled_page_tx, sizeof(oled_page_tx), OLED_I2C_TIMEOUT_MS) != HAL_OK)
    {
      oled_ready = false;
      return;
    }
  }
}

static int16_t Encoder_ComputeDelta(uint16_t current, uint16_t previous)
{
  int32_t delta = (int32_t)current - (int32_t)previous;

  if (delta > 32767L)
  {
    delta -= 65536L;
  }
  else if (delta < -32768L)
  {
    delta += 65536L;
  }

  return (int16_t)delta;
}

static uint8_t Buttons_ReadMask(void)
{
  uint8_t mask = 0U;

  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) { mask |= 0x01U; }
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) { mask |= 0x02U; }
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) { mask |= 0x04U; }
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET) { mask |= 0x08U; }

  return mask;
}

static void TestApp_PollButtons(void)
{
  uint8_t raw = Buttons_ReadMask();
  uint8_t page_button_pressed;
  uint32_t now = HAL_GetTick();

  if (raw != button_sample_mask)
  {
    button_sample_mask = raw;
    button_change_tick_ms = now;
  }

  if ((now - button_change_tick_ms) < BUTTON_DEBOUNCE_MS)
  {
    return;
  }

  if (button_stable_mask != button_sample_mask)
  {
    button_stable_mask = button_sample_mask;
    page_button_pressed = (uint8_t)(button_stable_mask & 0x08U);
    if ((page_button_pressed != 0U) && (button_prev_nonzero == 0U))
    {
      current_page = (test_page_t)((current_page + 1U) % TEST_PAGE_COUNT);
    }
    button_prev_nonzero = page_button_pressed;
  }
}

static void DrawStatusLine(uint8_t page, const char *label, int32_t value)
{
  char line[22];
  (void)snprintf(line, sizeof(line), "%-7s:%ld", label, (long)value);
  OLED_DrawString6x8(0U, page, line);
}

static uint16_t AdcToPwmPermille(uint16_t adc_value)
{
  uint32_t span = (uint32_t)(ADC_PWM_MAX_PERMILLE - ADC_PWM_MIN_PERMILLE);

  if (adc_value > 4095U)
  {
    adc_value = 4095U;
  }

  return (uint16_t)(ADC_PWM_MIN_PERMILLE + (((uint32_t)adc_value * span) / 4095U));
}

static uint16_t GetDrivePwmPermille(void)
{
  return AdcToPwmPermille(adc_buf[0]);
}

static uint16_t GetTurnPwmPermille(void)
{
  return AdcToPwmPermille(adc_buf[1]);
}

static uint16_t ClampPwmPermille(uint16_t value, uint16_t max_value)
{
  return (value > max_value) ? max_value : value;
}

static uint16_t GetObstacleSafeDistanceMm(void)
{
  uint32_t span = (uint32_t)(AUTO_MAPPING_MAX_SAFE_MM - AUTO_MAPPING_MIN_SAFE_MM);
  uint16_t adc_value = adc_buf[2];

  if (adc_value > 4095U)
  {
    adc_value = 4095U;
  }

  return (uint16_t)(AUTO_MAPPING_MIN_SAFE_MM + (((uint32_t)adc_value * span) / 4095U));
}

static uint16_t GetAutoFrontBlockDistanceMm(void)
{
  uint16_t safe_mm = GetObstacleSafeDistanceMm();

  return (safe_mm > AUTO_MAPPING_FRONT_BLOCK_MAX_MM) ?
      AUTO_MAPPING_FRONT_BLOCK_MAX_MM :
      safe_mm;
}

static bool PwmValueChanged(uint16_t current, uint16_t target)
{
  return (current > target) ?
      ((current - target) > ADC_PWM_UPDATE_DEADBAND) :
      ((target - current) > ADC_PWM_UPDATE_DEADBAND);
}

static int32_t NormalizeHeadingCdeg(int32_t heading_cdeg)
{
  while (heading_cdeg < 0L)
  {
    heading_cdeg += 36000L;
  }

  while (heading_cdeg >= 36000L)
  {
    heading_cdeg -= 36000L;
  }

  return heading_cdeg;
}

static int32_t AppAbs32(int32_t value)
{
  return (value < 0L) ? -value : value;
}

static void TestApp_UpdateGyroDriftCompensation(int16_t left_delta, int16_t right_delta)
{
  MotorControlState_t motor_state = {0};
  int32_t raw_gyro;
  int32_t diff;
  int32_t bias_step;
  bool stationary;
  bool motor_idle = true;

  if (gyro_calibration_active)
  {
    gyro_z_corrected_dps_x100 = 0L;
    return;
  }

  if (!mpu_state.ready)
  {
    gyro_z_corrected_dps_x100 = 0L;
    return;
  }

  raw_gyro = mpu_state.gyro_z_dps_x100;
  if (MotorControl_GetState(&motor_state))
  {
    motor_idle = (motor_state.active_pwm_mask == 0U);
  }

  stationary = ((AppAbs32((int32_t)left_delta) + AppAbs32((int32_t)right_delta)) <= GYRO_STATIONARY_COUNT_THRESHOLD);

  if (stationary && motor_idle)
  {
    if (gyro_bias_stationary_samples < GYRO_BIAS_STATIONARY_CONFIRM_SAMPLES)
    {
      gyro_bias_stationary_samples++;
    }
  }
  else
  {
    gyro_bias_stationary_samples = 0U;
  }

  if (gyro_bias_stationary_samples >= GYRO_BIAS_STATIONARY_CONFIRM_SAMPLES)
  {
    diff = raw_gyro - gyro_z_bias_dps_x100;
    bias_step = diff / GYRO_BIAS_FILTER_DIVISOR;
    if ((bias_step == 0L) && (diff != 0L))
    {
      bias_step = (diff > 0L) ? 1L : -1L;
    }
    gyro_z_bias_dps_x100 += bias_step;
  }

  gyro_z_corrected_dps_x100 = raw_gyro - gyro_z_bias_dps_x100;
  if (AppAbs32(gyro_z_corrected_dps_x100) <= GYRO_DEADBAND_DPS_X100)
  {
    gyro_z_corrected_dps_x100 = 0L;
  }
}

static void TestApp_StartGyroCalibration(void)
{
  gyro_calibration_active = true;
  gyro_calibration_start_tick_ms = HAL_GetTick();
  gyro_calibration_sum_dps_x100 = 0;
  gyro_calibration_samples = 0U;
  gyro_calibration_moving_samples = 0U;
  gyro_z_corrected_dps_x100 = 0L;
  gyro_bias_stationary_samples = 0U;

  (void)BluetoothControl_SendText("GYRO CAL START hold_still_ms=2000\r\n");
}

static void TestApp_UpdateGyroCalibration(uint32_t now_ms, int16_t left_delta, int16_t right_delta)
{
  bool stationary;
  bool done;
  char line[96];

  if (!gyro_calibration_active)
  {
    return;
  }

  if (!mpu_state.ready)
  {
    gyro_calibration_active = false;
    (void)BluetoothControl_SendText("GYRO CAL FAIL reason=MPU_NOT_READY\r\n");
    return;
  }

  stationary = ((AppAbs32((int32_t)left_delta) + AppAbs32((int32_t)right_delta)) <= GYRO_STATIONARY_COUNT_THRESHOLD);
  if (stationary)
  {
    gyro_calibration_sum_dps_x100 += (int64_t)mpu_state.gyro_z_dps_x100;
    if (gyro_calibration_samples < UINT16_MAX)
    {
      gyro_calibration_samples++;
    }
  }
  else if (gyro_calibration_moving_samples < UINT16_MAX)
  {
    gyro_calibration_moving_samples++;
  }

  done = ((now_ms - gyro_calibration_start_tick_ms) >= GYRO_CALIBRATION_DURATION_MS);
  if (!done)
  {
    return;
  }

  gyro_calibration_active = false;
  if ((gyro_calibration_samples < GYRO_CALIBRATION_MIN_SAMPLES) ||
      (gyro_calibration_moving_samples > GYRO_CALIBRATION_MAX_MOVING_SAMPLES))
  {
    gyro_bias_stationary_samples = 0U;
    (void)snprintf(
        line,
        sizeof(line),
        "GYRO CAL FAIL samples=%u moving=%u\r\n",
        (unsigned int)gyro_calibration_samples,
        (unsigned int)gyro_calibration_moving_samples);
    (void)BluetoothControl_SendText(line);
    return;
  }

  gyro_z_bias_dps_x100 = (int32_t)(gyro_calibration_sum_dps_x100 / (int64_t)gyro_calibration_samples);
  gyro_z_corrected_dps_x100 = 0L;
  gyro_bias_stationary_samples = GYRO_BIAS_STATIONARY_CONFIRM_SAMPLES;
  mapping_pose.heading_cdeg = MAPPING_START_HEADING_CDEG;
  odom_debug_heading_cdeg = 0L;
  MappingGrid_SetPose(&mapping_pose);

  (void)snprintf(
      line,
      sizeof(line),
      "GYRO CAL DONE bias=%ld samples=%u moving=%u\r\n",
      (long)gyro_z_bias_dps_x100,
      (unsigned int)gyro_calibration_samples,
      (unsigned int)gyro_calibration_moving_samples);
  (void)BluetoothControl_SendText(line);
}

static void RenderPage_LiDAR(void)
{
  uint32_t now = HAL_GetTick();
  bool stream_alive = false;

  if (lidar_result_valid && ((now - last_lidar_result_tick_ms) < 1000U) && (lidar_result.valid_nodes > 0U))
  {
    stream_alive = true;
  }

  OLED_DrawString6x8(0U, 0U, "LIDAR TEST");
  OLED_DrawString6x8(0U, 1U, (lidar_result_valid && (lidar_result.descriptor_seen != 0U)) ? "DESC   : YES" : "DESC   : NO");
  OLED_DrawString6x8(0U, 2U, mapping_active ? "MAP    : ON" : (stream_alive ? "STREAM : OK" : "STREAM : NO"));
  DrawStatusLine(3U, "BYTES", (int32_t)lidar_result.total_bytes);
  DrawStatusLine(4U, "VALID", (int32_t)lidar_result.valid_nodes);
  DrawStatusLine(5U, "INVAL", (int32_t)lidar_result.invalid_nodes);
  DrawStatusLine(6U, "DIST", (int32_t)lidar_result.last_distance_mm);
  DrawStatusLine(7U, "QUAL", (int32_t)lidar_result.last_quality);
}

static void RenderPage_MPU(void)
{
  char line[22];

  OLED_DrawString6x8(0U, 0U, "MPU6500 TEST");
  OLED_DrawString6x8(0U, 1U, mpu_state.ready ? "READY  : YES" : "READY  : NO");
  (void)snprintf(line, sizeof(line), "WHOAMI : %02X", mpu_state.who_am_i);
  OLED_DrawString6x8(0U, 2U, line);
  DrawStatusLine(3U, "GYROZ", (int32_t)mpu_state.gyro_z_raw);
  DrawStatusLine(4U, "DPSX100", mpu_state.gyro_z_dps_x100);
  OLED_DrawString6x8(0U, 6U, "BTN ANY TO PAGE");
  OLED_DrawString6x8(0U, 7U, "ADC PWM PAGE");
}

static void RenderPage_ADC(void)
{
  OLED_DrawString6x8(0U, 0U, "ADC TEST");
  DrawStatusLine(1U, "P1RAW", (int32_t)adc_buf[0]);
  DrawStatusLine(2U, "P2RAW", (int32_t)adc_buf[1]);
  DrawStatusLine(3U, "P3RAW", (int32_t)adc_buf[2]);
  DrawStatusLine(4U, "FWD", (int32_t)GetDrivePwmPermille());
  DrawStatusLine(5U, "TURN", (int32_t)GetTurnPwmPermille());
  DrawStatusLine(6U, "SAFE", (int32_t)GetObstacleSafeDistanceMm());
  OLED_DrawString6x8(0U, 7U, "P1FWD P2TURN P3SAFE");
}

static void RenderPage_Encoder(void)
{
  MotorControlState_t motor_state = {0};
  const char *mode_text = "MOTOR OFF";

  (void)MotorControl_GetState(&motor_state);

  if (motor_state.mode == 1U)
  {
    mode_text = "MOTOR FWD";
  }
  else if (motor_state.mode == 2U)
  {
    mode_text = "MOTOR LEFT";
  }
  else if (motor_state.mode == 3U)
  {
    mode_text = "MOTOR RIGHT";
  }
  else if (motor_state.mode == 4U)
  {
    mode_text = "MOTOR BRAKE";
  }

  OLED_DrawString6x8(0U, 0U, mode_text);
  DrawStatusLine(1U, "LDELTA", (int32_t)motor_state.left_delta);
  DrawStatusLine(2U, "RDELTA", (int32_t)motor_state.right_delta);
  DrawStatusLine(3U, "LDUTY", (int32_t)motor_state.left_duty_permille);
  DrawStatusLine(4U, "RDUTY", (int32_t)motor_state.right_duty_permille);
  DrawStatusLine(5U, "ERR", motor_state.balance_error);
  DrawStatusLine(6U, "CORR", motor_state.correction_permille);
  OLED_DrawString6x8(0U, 7U, "BT 0STOP 1FWD 2L 3R");
}

static void TestApp_UpdateDisplay(void)
{
  uint32_t now = HAL_GetTick();

  if (!oled_ready)
  {
    if ((now - last_oled_recover_tick_ms) >= OLED_RECOVER_RETRY_MS)
    {
      last_oled_recover_tick_ms = now;
      (void)HAL_I2C_DeInit(&hi2c2);
      MX_I2C2_Init();
      oled_ready = OLED_InitMinimal();
    }
    return;
  }

  if ((now - last_oled_tick_ms) < 150U)
  {
    return;
  }

  last_oled_tick_ms = now;
  OLED_Clear();

  switch (current_page)
  {
    case TEST_PAGE_LIDAR:
      RenderPage_LiDAR();
      break;
    case TEST_PAGE_MPU:
      RenderPage_MPU();
      break;
    case TEST_PAGE_ADC:
      RenderPage_ADC();
      break;
    case TEST_PAGE_ENCODER:
    default:
      RenderPage_Encoder();
      break;
  }

  OLED_Update();
}

static void TestApp_UpdateSensors(void)
{
  uint16_t left_now;
  uint16_t right_now;
  uint32_t now = HAL_GetTick();

  if ((now - last_sensor_tick_ms) < 20U)
  {
    return;
  }

  last_sensor_tick_ms = now;

  if (LidarPipeline_GetLatestResult(&lidar_result))
  {
    lidar_result_valid = true;
    last_lidar_result_tick_ms = now;
  }

  Mpu6500_Update();
  (void)Mpu6500_GetState(&mpu_state);

  left_now = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFU);
  right_now = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) & 0xFFFFU);

  encoder_test.left_delta = Encoder_ComputeDelta(left_now, encoder_test.last_left_counter);
  encoder_test.right_delta =
      (int16_t)((int32_t)Encoder_ComputeDelta(right_now, encoder_test.last_right_counter) * ENCODER_RIGHT_DELTA_SIGN);
  encoder_test.left_total += encoder_test.left_delta;
  encoder_test.right_total += encoder_test.right_delta;
  encoder_test.last_left_counter = left_now;
  encoder_test.last_right_counter = right_now;

  TestApp_UpdateGyroCalibration(now, encoder_test.left_delta, encoder_test.right_delta);
  TestApp_UpdateGyroDriftCompensation(encoder_test.left_delta, encoder_test.right_delta);
  TestApp_UpdateOdomDebug(now, encoder_test.left_delta, encoder_test.right_delta);
  TestApp_UpdateMappingPose(now, encoder_test.left_delta, encoder_test.right_delta);
  TestApp_UpdateAngleTurn(now);
  SlamNav_SetControlConfig(GetDrivePwmPermille(), GetTurnPwmPermille(), GetObstacleSafeDistanceMm());

  TestApp_UpdateAutoMapping(now);
  TestApp_UpdateMotorSpeedFromAdc();
}

static void TestApp_InitPeripherals(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_CHANNEL_COUNT);
  (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  if (!MotorControl_Init())
  {
    Error_Handler();
  }

  encoder_test.last_left_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFU);
  encoder_test.last_right_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) & 0xFFFFU);

  oled_ready = OLED_InitMinimal();
  (void)Mpu6500_Init();
  (void)Mpu6500_GetState(&mpu_state);
  (void)BluetoothControl_Init();
  MappingGrid_Init();
  (void)SlamNav_Init();
  if (!LidarPipeline_Init())
  {
    Error_Handler();
  }
}

static void TestApp_HandleBluetoothCommands(void)
{
  BluetoothCommand_t command;

  while (BluetoothControl_TakeCommand(&command))
  {
    switch (command.type)
    {
      case BLUETOOTH_CMD_DRIVE_FORWARD:
        SlamNav_Stop();
        TestApp_StopAutoMapping();
        TestApp_StopAngleTurn(false);
        MotorControl_SetForward(GetDrivePwmPermille());
        (void)BluetoothControl_SendText("MOTOR forward\r\n");
        break;

      case BLUETOOTH_CMD_TURN_LEFT:
        SlamNav_Stop();
        TestApp_StopAutoMapping();
        TestApp_StopAngleTurn(false);
        MotorControl_SetTurnLeft(GetTurnPwmPermille());
        (void)BluetoothControl_SendText("MOTOR left\r\n");
        break;

      case BLUETOOTH_CMD_TURN_RIGHT:
        SlamNav_Stop();
        TestApp_StopAutoMapping();
        TestApp_StopAngleTurn(false);
        MotorControl_SetTurnRight(GetTurnPwmPermille());
        (void)BluetoothControl_SendText("MOTOR right\r\n");
        break;

      case BLUETOOTH_CMD_DRIVE_STOP:
        SlamNav_Stop();
        TestApp_StopAutoMapping();
        TestApp_StopAngleTurn(false);
        MotorControl_Stop();
        (void)BluetoothControl_SendText("MOTOR brake\r\n");
        break;

      case BLUETOOTH_CMD_STOP_ALL:
        SlamNav_Stop();
        TestApp_StopAutoMapping();
        TestApp_StopAngleTurn(false);
        MotorControl_Stop();
        TestApp_StopMapping();
        lidar_debug_active = false;
        TestApp_StopOdomDebug();
        (void)BluetoothControl_SendText("EMERGENCY STOP\r\n");
        break;

      case BLUETOOTH_CMD_START_MAPPING:
        SlamNav_Stop();
        TestApp_StartMapping();
        break;

      case BLUETOOTH_CMD_STOP_MAPPING:
        TestApp_StopMapping();
        (void)BluetoothControl_SendText("MAP STOP\r\n");
        break;

      case BLUETOOTH_CMD_LIDAR_DEBUG_ON:
        lidar_debug_active = true;
        lidar_debug_seq = 0U;
        (void)BluetoothControl_SendText("LIDAR START format=LP seq=<n> a=<raw_cdeg> r=<robot_cdeg> d=<mm> q=<quality> s=<scan_start>\r\n");
        break;

      case BLUETOOTH_CMD_LIDAR_DEBUG_OFF:
        lidar_debug_active = false;
        (void)BluetoothControl_SendText("LIDAR STOP\r\n");
        break;

      case BLUETOOTH_CMD_LIDAR_QUALITY_SET:
        TestApp_SetLidarQualityFilter(command.text);
        break;

      case BLUETOOTH_CMD_ODOM_DEBUG_ON:
        TestApp_StartOdomDebug();
        break;

      case BLUETOOTH_CMD_ODOM_DEBUG_OFF:
        TestApp_StopOdomDebug();
        break;

      case BLUETOOTH_CMD_ENCODER_CAL_END:
        TestApp_EndEncoderCalibration();
        break;

      case BLUETOOTH_CMD_AUTO_MAPPING_ON:
        SlamNav_Stop();
        TestApp_StartAutoMapping();
        break;

      case BLUETOOTH_CMD_AUTO_MAPPING_OFF:
        SlamNav_Stop();
        TestApp_StopAutoMapping();
        (void)BluetoothControl_SendText("AUTO MAP STOP\r\n");
        break;

      case BLUETOOTH_CMD_SLAM_NAV_ON:
        TestApp_StopAutoMapping();
        TestApp_StopAngleTurn(false);
        MotorControl_Stop();
        TestApp_StartMapping();
        SlamNav_SetControlConfig(GetDrivePwmPermille(), GetTurnPwmPermille(), GetObstacleSafeDistanceMm());
        SlamNav_StartExplore();
        break;

      case BLUETOOTH_CMD_SLAM_NAV_OFF:
        SlamNav_Stop();
        TestApp_StopMapping();
        (void)BluetoothControl_SendText("SLAM STOP\r\n");
        break;

      case BLUETOOTH_CMD_SLAM_NAV_RETURN:
        TestApp_StopAutoMapping();
        TestApp_StopAngleTurn(false);
        MotorControl_Stop();
        TestApp_ResumeMapping();
        SlamNav_SetControlConfig(GetDrivePwmPermille(), GetTurnPwmPermille(), GetObstacleSafeDistanceMm());
        SlamNav_StartReturnTo(MAPPING_START_X_MM, MAPPING_START_Y_MM);
        break;

      case BLUETOOTH_CMD_GYRO_CALIBRATE:
        SlamNav_Stop();
        TestApp_StopAutoMapping();
        TestApp_StopAngleTurn(false);
        MotorControl_Stop();
        TestApp_StopMapping();
        TestApp_StartGyroCalibration();
        break;

      case BLUETOOTH_CMD_MPU_STATE:
        TestApp_SendMpuState();
        break;

      case BLUETOOTH_CMD_LIDAR_FRONT_STATE:
        TestApp_SendLidarFrontState();
        break;

      case BLUETOOTH_CMD_DIR_RESET:
        TestApp_ResetMapNorth();
        break;

      case BLUETOOTH_CMD_DIR_STATE:
        TestApp_SendDirState();
        break;

      case BLUETOOTH_CMD_TURN_LEFT_DEG:
        SlamNav_Stop();
        TestApp_StopAutoMapping();
        TestApp_StartAngleTurn(-1, TestApp_ParseTurnDegrees(command.text));
        break;

      case BLUETOOTH_CMD_TURN_RIGHT_DEG:
        SlamNav_Stop();
        TestApp_StopAutoMapping();
        TestApp_StartAngleTurn(1, TestApp_ParseTurnDegrees(command.text));
        break;

      case BLUETOOTH_CMD_SHOW_MAP_RESULT:
        TestApp_RequestFullMapStream();
        (void)BluetoothControl_SendText("MAP SHOW\r\n");
        break;

      case BLUETOOTH_CMD_NONE:
      case BLUETOOTH_CMD_DEBUG_ON:
      case BLUETOOTH_CMD_DEBUG_OFF:
      case BLUETOOTH_CMD_UNKNOWN:
      default:
        break;
    }
  }
}

static void TestApp_StartMapping(void)
{
  mapping_pose.x_mm = MAPPING_START_X_MM;
  mapping_pose.y_mm = MAPPING_START_Y_MM;
  mapping_pose.heading_cdeg = MAPPING_START_HEADING_CDEG;
  mapping_travel_residual_x1000 = 0L;
  MappingGrid_Reset();
  MappingGrid_SetPose(&mapping_pose);
  MappingGrid_MarkRobotFree(&mapping_pose, 1U);
  TestApp_ResetPoseHistory();

  mapping_active = true;
  last_mapping_pose_tick_ms = HAL_GetTick();
  mapping_start_tick_ms = last_mapping_pose_tick_ms;
  TestApp_RecordPoseHistory(last_mapping_pose_tick_ms);
  last_map_row_tx_tick_ms = last_mapping_pose_tick_ms;
  last_map_stat_tx_tick_ms = last_mapping_pose_tick_ms;
  last_pose_tx_tick_ms = last_mapping_pose_tick_ms;
  next_map_tx_row = 0U;

  TestApp_SendMapHeader("START");
}

static void TestApp_ResumeMapping(void)
{
  uint32_t now = HAL_GetTick();

  if (mapping_active)
  {
    return;
  }

  mapping_active = true;
  last_mapping_pose_tick_ms = now;
  if (mapping_start_tick_ms == 0U)
  {
    mapping_start_tick_ms = now;
  }
  TestApp_RecordPoseHistory(now);
  last_map_row_tx_tick_ms = now;
  last_map_stat_tx_tick_ms = now;
  last_pose_tx_tick_ms = now;
  next_map_tx_row = 0U;

  TestApp_SendMapHeader("RESUME");
}

static void TestApp_StopMapping(void)
{
  if (!mapping_active)
  {
    return;
  }

  mapping_active = false;
  TestApp_SendMapHeader("STOP");
  TestApp_SendMapStat();
}

static void TestApp_ProcessLidarPoints(void)
{
  LidarPoint_t point;
  MappingGridPose_t point_pose;
  uint32_t now_ms = HAL_GetTick();
  uint8_t count = 0U;
  uint8_t debug_tx_count = 0U;

  while ((count < MAPPING_POINT_BATCH_LIMIT) && LidarPipeline_TakePoint(&point))
  {
    bool point_is_fresh = ((int32_t)(point.tick_count - mapping_start_tick_ms) >= 0L) &&
        ((now_ms - point.tick_count) <= MAPPING_LIDAR_POINT_MAX_AGE_MS);

    if (point_is_fresh)
    {
      TestApp_UpdateLidarFrontStats(&point);
      TestApp_UpdateAutoMappingObstacle(&point);
      SlamNav_ObserveLidarPoint(&point);
    }

    if (mapping_active)
    {
      if (point_is_fresh && TestApp_GetPoseForTick(point.tick_count, &point_pose))
      {
        (void)MappingGrid_InsertPolarPointAtPose(&point_pose, point.angle_cdeg, point.distance_mm, point.quality);
      }
      else if (point_is_fresh)
      {
        (void)MappingGrid_InsertPolarPoint(point.angle_cdeg, point.distance_mm, point.quality);
      }
    }

    if (lidar_debug_active &&
        ((debug_tx_count < LIDAR_DEBUG_MAX_TX_PER_BATCH) ||
         ((point.flags & LIDAR_POINT_FLAG_SCAN_START) != 0U)))
    {
      TestApp_SendLidarDebugPoint(&point);
      debug_tx_count++;
    }

    count++;
  }
}

static void TestApp_SendLidarDebugPoint(const LidarPoint_t *point)
{
  char line[96];

  if (point == NULL)
  {
    return;
  }

  (void)snprintf(
      line,
      sizeof(line),
      "LP seq=%lu a=%u r=%u d=%u q=%u s=%u\r\n",
      (unsigned long)lidar_debug_seq++,
      (unsigned int)point->angle_cdeg,
      (unsigned int)LidarPipeline_LidarToRobotAngleU16(point->angle_cdeg),
      (unsigned int)point->distance_mm,
      (unsigned int)point->quality,
      (unsigned int)((point->flags & LIDAR_POINT_FLAG_SCAN_START) != 0U));
  (void)BluetoothControl_SendText(line);
}

static void TestApp_UpdateLidarFrontStats(const LidarPoint_t *point)
{
  uint16_t robot_angle_cdeg;
  uint16_t clearance_mm;

  if (point == NULL)
  {
    return;
  }

  if ((point->flags & LIDAR_POINT_FLAG_SCAN_START) != 0U)
  {
    if (lidar_front_current.point_count > 0U)
    {
      lidar_front_last = lidar_front_current;
    }
    lidar_front_scan_seq++;
    TestApp_ResetLidarFrontStats(&lidar_front_current, lidar_front_scan_seq);
  }

  robot_angle_cdeg = LidarPipeline_LidarToRobotAngleU16(point->angle_cdeg);
  if (!TestApp_IsFrontLidarPoint(robot_angle_cdeg))
  {
    return;
  }
  clearance_mm = TestApp_ApplyForwardBodyOffset(robot_angle_cdeg, point->distance_mm);

  lidar_front_current.point_count++;
  lidar_front_current.last_tick_ms = HAL_GetTick();
  if ((lidar_front_current.min_distance_mm == 0U) ||
      (clearance_mm < lidar_front_current.min_distance_mm))
  {
    lidar_front_current.min_tick_ms = lidar_front_current.last_tick_ms;
    lidar_front_current.min_distance_mm = clearance_mm;
    lidar_front_current.min_raw_angle_cdeg = point->angle_cdeg;
    lidar_front_current.min_robot_angle_cdeg = robot_angle_cdeg;
    lidar_front_current.min_quality = point->quality;
  }
}

static void TestApp_ResetLidarFrontStats(lidar_front_stats_t *stats, uint32_t scan_seq)
{
  if (stats == NULL)
  {
    return;
  }

  memset(stats, 0, sizeof(*stats));
  stats->scan_seq = scan_seq;
}

static void TestApp_SendLidarFrontState(void)
{
  lidar_front_stats_t stats = lidar_front_last;
  uint32_t now = HAL_GetTick();
  uint32_t age_ms = 0U;
  uint16_t safe_mm = GetObstacleSafeDistanceMm();
  uint16_t block_mm = GetAutoFrontBlockDistanceMm();
  bool data_valid;
  bool direct_blocked;
  bool auto_blocked;
  char line[192];

  if (lidar_front_current.scan_seq >= stats.scan_seq)
  {
    stats = lidar_front_current;
  }

  if (stats.min_tick_ms != 0U)
  {
    age_ms = now - stats.min_tick_ms;
  }

  data_valid = ((stats.point_count > 0U) &&
      (stats.min_distance_mm > 0U) &&
      (age_ms <= LIDAR_FRONT_STATE_MAX_AGE_MS));
  direct_blocked = data_valid && (stats.min_distance_mm <= block_mm);
  auto_blocked = TestApp_IsAutoFrontBlocked(now);

  (void)snprintf(
      line,
      sizeof(line),
      "LIDAR FRONT scan=%lu cnt=%u min=%u raw=%u robot=%u q=%u age=%lu safe=%u block=%u sector=%u body=%u blocked=%u auto=%u\r\n",
      (unsigned long)stats.scan_seq,
      (unsigned int)stats.point_count,
      (unsigned int)stats.min_distance_mm,
      (unsigned int)stats.min_raw_angle_cdeg,
      (unsigned int)stats.min_robot_angle_cdeg,
      (unsigned int)stats.min_quality,
      (unsigned long)age_ms,
      (unsigned int)safe_mm,
      (unsigned int)block_mm,
      (unsigned int)AUTO_MAPPING_FRONT_SECTOR_CDEG,
      (unsigned int)AUTO_MAPPING_FRONT_BODY_OFFSET_MM,
      (unsigned int)direct_blocked,
      (unsigned int)auto_blocked);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_UpdateMappingPose(uint32_t now_ms, int16_t left_delta, int16_t right_delta)
{
  MotorControlState_t motor_state = {0};
  uint32_t delta_ms;
  int32_t gyro_delta_cdeg = 0L;
  int32_t heading_for_travel_cdeg;
  int32_t travel_counts;
  int32_t travel_x1000;
  int32_t travel_mm;
  float heading_rad;

  if (!mapping_active)
  {
    if (!auto_mapping_active && !angle_turn_active && !SlamNav_IsActive())
    {
      last_mapping_pose_tick_ms = now_ms;
      return;
    }
  }

  if (last_mapping_pose_tick_ms == 0U)
  {
    last_mapping_pose_tick_ms = now_ms;
    if (mapping_active)
    {
      MappingGrid_SetPose(&mapping_pose);
    }
    return;
  }

  delta_ms = now_ms - last_mapping_pose_tick_ms;
  last_mapping_pose_tick_ms = now_ms;

  if (mpu_state.ready)
  {
    gyro_delta_cdeg = (gyro_z_corrected_dps_x100 * (int32_t)delta_ms) / 1000L;
    mapping_pose.heading_cdeg = NormalizeHeadingCdeg(mapping_pose.heading_cdeg + gyro_delta_cdeg);
  }

  if (MotorControl_GetState(&motor_state) && motor_state.forward_active)
  {
    travel_counts = (AppAbs32((int32_t)left_delta) + AppAbs32((int32_t)right_delta)) / 2L;
    travel_x1000 = (travel_counts * mapping_encoder_mm_per_count_x1000) + mapping_travel_residual_x1000;
    travel_mm = travel_x1000 / 1000L;
    mapping_travel_residual_x1000 = travel_x1000 - (travel_mm * 1000L);
    if (travel_mm != 0L)
    {
      heading_for_travel_cdeg = NormalizeHeadingCdeg(mapping_pose.heading_cdeg - (gyro_delta_cdeg / 2L));
      heading_rad = ((float)heading_for_travel_cdeg * MAPPING_PI) / 18000.0f;
      mapping_pose.x_mm += (int32_t)((float)travel_mm * cosf(heading_rad));
      mapping_pose.y_mm += (int32_t)((float)travel_mm * sinf(heading_rad));
    }
  }

  if (mapping_active)
  {
    MappingGrid_SetPose(&mapping_pose);
    TestApp_RecordPoseHistory(now_ms);
  }
}

static void TestApp_ResetPoseHistory(void)
{
  memset(mapping_pose_history, 0, sizeof(mapping_pose_history));
  memset(mapping_pose_history_ticks, 0, sizeof(mapping_pose_history_ticks));
  mapping_pose_history_next = 0U;
  mapping_pose_history_count = 0U;
}

static void TestApp_RecordPoseHistory(uint32_t tick_ms)
{
  mapping_pose_history[mapping_pose_history_next] = mapping_pose;
  mapping_pose_history_ticks[mapping_pose_history_next] = tick_ms;
  mapping_pose_history_next++;
  if (mapping_pose_history_next >= MAPPING_POSE_HISTORY_LENGTH)
  {
    mapping_pose_history_next = 0U;
  }

  if (mapping_pose_history_count < MAPPING_POSE_HISTORY_LENGTH)
  {
    mapping_pose_history_count++;
  }
}

static bool TestApp_GetPoseForTick(uint32_t tick_ms, MappingGridPose_t *out_pose)
{
  uint8_t i;
  uint8_t best_index = 0U;
  uint32_t best_error = UINT32_MAX;

  if ((out_pose == NULL) || (mapping_pose_history_count == 0U))
  {
    return false;
  }

  for (i = 0U; i < mapping_pose_history_count; ++i)
  {
    uint32_t pose_tick = mapping_pose_history_ticks[i];
    uint32_t error = (tick_ms >= pose_tick) ? (tick_ms - pose_tick) : (pose_tick - tick_ms);

    if (error < best_error)
    {
      best_error = error;
      best_index = i;
    }
  }

  *out_pose = mapping_pose_history[best_index];
  return true;
}

static void TestApp_StartOdomDebug(void)
{
  uint32_t now = HAL_GetTick();

  odom_debug_active = true;
  odom_debug_seq = 0U;
  odom_debug_left_counts = 0L;
  odom_debug_right_counts = 0L;
  odom_debug_heading_cdeg = 0L;
  last_odom_debug_update_tick_ms = now;
  last_odom_debug_tx_tick_ms = 0U;

  (void)BluetoothControl_SendText("ODOM START cal_mm=350 push_straight_then_send_END_ENCODER fields=seq,lc,rc,avg,dist,cal_k,heading,gz,gb,k\r\n");
}

static void TestApp_StopOdomDebug(void)
{
  if (!odom_debug_active)
  {
    return;
  }

  odom_debug_active = false;
  (void)BluetoothControl_SendText("ODOM STOP\r\n");
}

static void TestApp_EndEncoderCalibration(void)
{
  char line[192];
  int32_t abs_left;
  int32_t abs_right;
  int32_t average_counts;
  int32_t new_k_x1000;
  int32_t old_k_x1000 = mapping_encoder_mm_per_count_x1000;

  if (!odom_debug_active)
  {
    (void)BluetoothControl_SendText("ODOM CAL ERR inactive send_94_first\r\n");
    return;
  }

  abs_left = AppAbs32(odom_debug_left_counts);
  abs_right = AppAbs32(odom_debug_right_counts);
  average_counts = (abs_left + abs_right) / 2L;
  odom_debug_active = false;

  if (average_counts < ENCODER_CAL_MIN_COUNTS)
  {
    (void)snprintf(
        line,
        sizeof(line),
        "ODOM CAL ERR avg=%ld min=%ld lc=%ld rc=%ld k=%ld\r\n",
        (long)average_counts,
        (long)ENCODER_CAL_MIN_COUNTS,
        (long)odom_debug_left_counts,
        (long)odom_debug_right_counts,
        (long)old_k_x1000);
    (void)BluetoothControl_SendText(line);
    return;
  }

  new_k_x1000 = (ENCODER_CAL_DISTANCE_MM * 1000L) / average_counts;
  mapping_encoder_mm_per_count_x1000 = new_k_x1000;
  mapping_travel_residual_x1000 = 0L;

  (void)snprintf(
      line,
      sizeof(line),
      "ODOM CAL DONE cal_mm=%ld avg=%ld old_k=%ld new_k=%ld lc=%ld rc=%ld persist=ram\r\n",
      (long)ENCODER_CAL_DISTANCE_MM,
      (long)average_counts,
      (long)old_k_x1000,
      (long)new_k_x1000,
      (long)odom_debug_left_counts,
      (long)odom_debug_right_counts);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_UpdateOdomDebug(uint32_t now_ms, int16_t left_delta, int16_t right_delta)
{
  uint32_t delta_ms;

  if (!odom_debug_active)
  {
    last_odom_debug_update_tick_ms = now_ms;
    return;
  }

  if (last_odom_debug_update_tick_ms == 0U)
  {
    last_odom_debug_update_tick_ms = now_ms;
    return;
  }

  delta_ms = now_ms - last_odom_debug_update_tick_ms;
  last_odom_debug_update_tick_ms = now_ms;

  odom_debug_left_counts += (int32_t)left_delta;
  odom_debug_right_counts += (int32_t)right_delta;

  if (mpu_state.ready)
  {
    odom_debug_heading_cdeg += (gyro_z_corrected_dps_x100 * (int32_t)delta_ms) / 1000L;
  }
}

static void TestApp_StreamOdomDebug(void)
{
  char line[192];
  uint32_t now = HAL_GetTick();
  int32_t abs_left;
  int32_t abs_right;
  int32_t average_counts;
  int32_t distance_mm;
  int32_t cal_k_x1000 = 0L;

  if (!odom_debug_active)
  {
    return;
  }

  if ((now - last_odom_debug_tx_tick_ms) < ODOM_DEBUG_TX_INTERVAL_MS)
  {
    return;
  }
  last_odom_debug_tx_tick_ms = now;

  abs_left = AppAbs32(odom_debug_left_counts);
  abs_right = AppAbs32(odom_debug_right_counts);
  average_counts = (abs_left + abs_right) / 2L;
  distance_mm = (average_counts * mapping_encoder_mm_per_count_x1000) / 1000L;
  if (average_counts > 0L)
  {
    cal_k_x1000 = (ENCODER_CAL_DISTANCE_MM * 1000L) / average_counts;
  }

  (void)snprintf(
      line,
      sizeof(line),
      "ODOM seq=%lu lc=%ld rc=%ld avg=%ld dist=%ld cal_k=%ld heading=%ld gz=%ld gb=%ld k=%ld\r\n",
      (unsigned long)odom_debug_seq++,
      (long)odom_debug_left_counts,
      (long)odom_debug_right_counts,
      (long)average_counts,
      (long)distance_mm,
      (long)cal_k_x1000,
      (long)odom_debug_heading_cdeg,
      (long)gyro_z_corrected_dps_x100,
      (long)gyro_z_bias_dps_x100,
      (long)mapping_encoder_mm_per_count_x1000);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_SetLidarQualityFilter(const char *text)
{
  char line[80];
  uint8_t min_quality = TestApp_ParseLidarQuality(text, LidarPipeline_GetMinPointQuality());

  LidarPipeline_SetMinPointQuality(min_quality);
  (void)snprintf(
      line,
      sizeof(line),
      "LIDAR QUALITY min=%u\r\n",
      (unsigned int)LidarPipeline_GetMinPointQuality());
  (void)BluetoothControl_SendText(line);
}

static uint16_t TestApp_ParseTurnDegrees(const char *text)
{
  uint32_t value = 0U;
  bool has_digit = false;

  if (text == NULL)
  {
    return ANGLE_TURN_DEFAULT_DEG;
  }

  while (*text != '\0')
  {
    if ((*text >= '0') && (*text <= '9'))
    {
      has_digit = true;
      value = (value * 10U) + (uint32_t)(*text - '0');
      if (value > ANGLE_TURN_MAX_DEG)
      {
        value = ANGLE_TURN_MAX_DEG;
        break;
      }
    }
    text++;
  }

  if (!has_digit)
  {
    value = ANGLE_TURN_DEFAULT_DEG;
  }
  else if (value < ANGLE_TURN_MIN_DEG)
  {
    value = ANGLE_TURN_MIN_DEG;
  }

  return (uint16_t)value;
}

static uint8_t TestApp_ParseLidarQuality(const char *text, uint8_t fallback)
{
  uint32_t value = 0U;
  bool has_digit = false;

  if (text == NULL)
  {
    return fallback;
  }

  while (*text != '\0')
  {
    if ((*text >= '0') && (*text <= '9'))
    {
      has_digit = true;
      value = (value * 10U) + (uint32_t)(*text - '0');
      if (value > 63U)
      {
        value = 63U;
        break;
      }
    }
    text++;
  }

  return has_digit ? (uint8_t)value : fallback;
}

static void TestApp_StartAngleTurn(int8_t direction, uint16_t degrees)
{
  char line[80];
  uint16_t turn_pwm;

  if (degrees < ANGLE_TURN_MIN_DEG)
  {
    degrees = ANGLE_TURN_MIN_DEG;
  }
  else if (degrees > ANGLE_TURN_MAX_DEG)
  {
    degrees = ANGLE_TURN_MAX_DEG;
  }

  angle_turn_active = true;
  angle_turn_direction = (direction < 0) ? -1 : 1;
  angle_turn_heading_target_valid = false;
  angle_turn_completed_by_progress = false;
  angle_turn_correction_count = 0U;
  angle_turn_target_cdeg = (int32_t)degrees * 100L;
  angle_turn_target_heading_cdeg = 0L;
  angle_turn_progress_cdeg = 0L;
  angle_turn_start_tick_ms = HAL_GetTick();
  angle_turn_last_tick_ms = angle_turn_start_tick_ms;

  turn_pwm = ClampPwmPermille(GetTurnPwmPermille(), AUTO_MAPPING_MAX_TURN_PWM);
  if (angle_turn_direction < 0)
  {
    MotorControl_SetTurnLeft(turn_pwm);
  }
  else
  {
    MotorControl_SetTurnRight(turn_pwm);
  }

  (void)snprintf(
      line,
      sizeof(line),
      "TURN START dir=%c deg=%u target=%ld\r\n",
      (angle_turn_direction < 0) ? 'L' : 'R',
      (unsigned int)degrees,
      (long)angle_turn_target_cdeg);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_StartHeadingTurn(int32_t target_heading_cdeg, uint8_t correction_count, const char *reason)
{
  char line[96];
  int32_t error_cdeg;
  uint16_t degrees;
  int8_t direction;

  target_heading_cdeg = NormalizeHeadingCdeg(target_heading_cdeg);
  error_cdeg = TestApp_SignedHeadingErrorCdeg(target_heading_cdeg, mapping_pose.heading_cdeg);
  degrees = (uint16_t)((AppAbs32(error_cdeg) + 50L) / 100L);
  if (degrees < ANGLE_TURN_MIN_DEG)
  {
    degrees = ANGLE_TURN_MIN_DEG;
  }
  direction = (error_cdeg >= 0L) ? -1 : 1;

  TestApp_StartAngleTurn(direction, degrees);
  angle_turn_heading_target_valid = true;
  angle_turn_correction_count = correction_count;
  angle_turn_target_heading_cdeg = target_heading_cdeg;

  if (reason == NULL)
  {
    reason = "TARGET";
  }

  (void)snprintf(
      line,
      sizeof(line),
      "TURN HEADING %s target=%ld current=%ld err=%ld corr=%u\r\n",
      reason,
      (long)angle_turn_target_heading_cdeg,
      (long)mapping_pose.heading_cdeg,
      (long)error_cdeg,
      (unsigned int)angle_turn_correction_count);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_StopAngleTurn(bool completed)
{
  char line[80];
  bool heading_target_valid;
  bool completed_by_progress;
  uint8_t correction_count;
  int32_t target_heading_cdeg;
  int32_t heading_error_cdeg = 0L;

  if (!angle_turn_active)
  {
    return;
  }

  heading_target_valid = angle_turn_heading_target_valid;
  completed_by_progress = angle_turn_completed_by_progress;
  correction_count = angle_turn_correction_count;
  target_heading_cdeg = angle_turn_target_heading_cdeg;
  if (heading_target_valid)
  {
    heading_error_cdeg = TestApp_SignedHeadingErrorCdeg(target_heading_cdeg, mapping_pose.heading_cdeg);
  }

  angle_turn_active = false;
  angle_turn_direction = 0;
  angle_turn_heading_target_valid = false;
  angle_turn_completed_by_progress = false;
  MotorControl_Stop();

  if (completed && heading_target_valid && completed_by_progress)
  {
    mapping_pose.heading_cdeg = NormalizeHeadingCdeg(target_heading_cdeg);
    if (mapping_active)
    {
      MappingGrid_SetPose(&mapping_pose);
    }
    heading_error_cdeg = 0L;
  }

  if (completed &&
      heading_target_valid &&
      !completed_by_progress &&
      (AppAbs32(heading_error_cdeg) > ANGLE_TURN_CORRECTION_TOL_CDEG) &&
      (correction_count < ANGLE_TURN_MAX_CORRECTIONS))
  {
    TestApp_StartHeadingTurn(target_heading_cdeg, (uint8_t)(correction_count + 1U), "CORRECT");
    return;
  }

  if (auto_mapping_active)
  {
    auto_mapping_resume_tick_ms = HAL_GetTick() + AUTO_MAPPING_TURN_SETTLE_MS;
    auto_mapping_observe_after_resume = true;
    if (completed)
    {
      auto_mapping_ignore_right_until_ms = auto_mapping_resume_tick_ms + AUTO_MAPPING_POST_TURN_DRIVE_MS;
    }
  }

  (void)snprintf(
      line,
      sizeof(line),
      "TURN %s progress=%ld target=%ld herr=%ld\r\n",
      completed ? "DONE" : "STOP",
      (long)angle_turn_progress_cdeg,
      (long)angle_turn_target_cdeg,
      (long)heading_error_cdeg);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_UpdateAngleTurn(uint32_t now_ms)
{
  uint32_t delta_ms;
  int32_t delta_cdeg;
  int32_t remaining_cdeg;
  int32_t progress_remaining_cdeg;
  int32_t heading_remaining_cdeg;
  uint16_t turn_pwm;

  if (!angle_turn_active)
  {
    angle_turn_last_tick_ms = now_ms;
    return;
  }

  if (angle_turn_last_tick_ms == 0U)
  {
    angle_turn_last_tick_ms = now_ms;
    return;
  }

  delta_ms = now_ms - angle_turn_last_tick_ms;
  angle_turn_last_tick_ms = now_ms;
  delta_cdeg = (gyro_z_corrected_dps_x100 * (int32_t)delta_ms) / 1000L;
  angle_turn_progress_cdeg += AppAbs32(delta_cdeg);
  progress_remaining_cdeg = angle_turn_target_cdeg - angle_turn_progress_cdeg;

  if (angle_turn_heading_target_valid)
  {
    heading_remaining_cdeg = AppAbs32(TestApp_SignedHeadingErrorCdeg(
        angle_turn_target_heading_cdeg,
        mapping_pose.heading_cdeg));
    remaining_cdeg = (progress_remaining_cdeg < heading_remaining_cdeg) ?
        progress_remaining_cdeg :
        heading_remaining_cdeg;
  }
  else
  {
    heading_remaining_cdeg = 0L;
    remaining_cdeg = progress_remaining_cdeg;
  }

  if (remaining_cdeg <= ANGLE_TURN_DONE_TOL_CDEG)
  {
    angle_turn_completed_by_progress =
        angle_turn_heading_target_valid &&
        (progress_remaining_cdeg <= ANGLE_TURN_DONE_TOL_CDEG) &&
        (heading_remaining_cdeg > ANGLE_TURN_DONE_TOL_CDEG);
    TestApp_StopAngleTurn(true);
    return;
  }

  if ((now_ms - angle_turn_start_tick_ms) >= ANGLE_TURN_TIMEOUT_MS)
  {
    TestApp_StopAngleTurn(false);
    return;
  }

  turn_pwm = ClampPwmPermille(GetTurnPwmPermille(), AUTO_MAPPING_MAX_TURN_PWM);
  if ((remaining_cdeg <= ANGLE_TURN_FINE_ZONE_CDEG) && (turn_pwm > ANGLE_TURN_FINE_PWM))
  {
    turn_pwm = ANGLE_TURN_FINE_PWM;
  }
  else if ((remaining_cdeg <= ANGLE_TURN_SLOW_ZONE_CDEG) && (turn_pwm > ANGLE_TURN_SLOW_PWM))
  {
    turn_pwm = ANGLE_TURN_SLOW_PWM;
  }

  if (angle_turn_direction < 0)
  {
    MotorControl_SetTurnLeft(turn_pwm);
  }
  else
  {
    MotorControl_SetTurnRight(turn_pwm);
  }
}

static void TestApp_StartAutoMapping(void)
{
  auto_mapping_active = true;
  TestApp_ResetAutoMappingSectorMins();
  last_auto_mapping_control_tick_ms = 0U;
  auto_mapping_avoid_count = 0U;
  auto_mapping_resume_tick_ms = 0U;
  auto_mapping_ignore_right_until_ms = 0U;
  auto_mapping_observe_deadline_ms = 0U;
  auto_mapping_front_blocked_until_ms = 0U;
  auto_mapping_front_last_seen_ms = 0U;
  auto_mapping_front_last_robot_angle_cdeg = 0U;
  auto_mapping_front_last_raw_angle_cdeg = 0U;
  auto_mapping_right_branch_confirm_count = 0U;
  auto_mapping_right_wall_seen = false;
  auto_mapping_observe_after_resume = false;

  TestApp_StartAutoObservation("START");
  (void)BluetoothControl_SendText("AUTO WALL START rule=right-hand\r\n");
}

static void TestApp_StopAutoMapping(void)
{
  if (!auto_mapping_active)
  {
    return;
  }

  auto_mapping_active = false;
  TestApp_ResetAutoMappingSectorMins();
  auto_mapping_resume_tick_ms = 0U;
  auto_mapping_ignore_right_until_ms = 0U;
  auto_mapping_observe_deadline_ms = 0U;
  auto_mapping_front_blocked_until_ms = 0U;
  auto_mapping_front_last_seen_ms = 0U;
  auto_mapping_front_last_robot_angle_cdeg = 0U;
  auto_mapping_front_last_raw_angle_cdeg = 0U;
  auto_mapping_right_branch_confirm_count = 0U;
  auto_mapping_observe_scan_starts_remaining = 0U;
  auto_mapping_right_wall_seen = false;
  auto_mapping_observe_after_resume = false;
  MotorControl_Stop();
}

static bool TestApp_IsFrontLidarPoint(uint16_t angle_cdeg)
{
  return ((angle_cdeg <= AUTO_MAPPING_FRONT_SECTOR_CDEG) ||
          (angle_cdeg >= (uint16_t)(36000U - AUTO_MAPPING_FRONT_SECTOR_CDEG)));
}

static bool TestApp_IsLidarAngleNear(uint16_t angle_cdeg, uint16_t center_cdeg, uint16_t half_width_cdeg)
{
  int32_t diff = (int32_t)angle_cdeg - (int32_t)center_cdeg;

  if (diff > 18000L)
  {
    diff -= 36000L;
  }
  else if (diff < -18000L)
  {
    diff += 36000L;
  }

  return AppAbs32(diff) <= (int32_t)half_width_cdeg;
}

static uint16_t TestApp_ApplyForwardBodyOffset(uint16_t robot_angle_cdeg, uint16_t distance_mm)
{
  uint16_t offset_mm = 0U;

  if (distance_mm == 0U)
  {
    return 0U;
  }

  if (TestApp_IsFrontLidarPoint(robot_angle_cdeg))
  {
    offset_mm = AUTO_MAPPING_FRONT_BODY_OFFSET_MM;
  }
  else if (TestApp_IsLidarAngleNear(robot_angle_cdeg, AUTO_MAPPING_FRONT_RIGHT_CENTER_CDEG, AUTO_MAPPING_DIAGONAL_SECTOR_CDEG) ||
           TestApp_IsLidarAngleNear(robot_angle_cdeg, AUTO_MAPPING_FRONT_LEFT_CENTER_CDEG, AUTO_MAPPING_DIAGONAL_SECTOR_CDEG))
  {
    offset_mm = AUTO_MAPPING_FRONT_DIAGONAL_BODY_OFFSET_MM;
  }

  if (offset_mm == 0U)
  {
    return distance_mm;
  }

  return (distance_mm > offset_mm) ? (uint16_t)(distance_mm - offset_mm) : 1U;
}

static void TestApp_UpdateAutoMappingObstacle(const LidarPoint_t *point)
{
  uint16_t robot_angle_cdeg;
  uint16_t clearance_mm;

  if ((point == NULL) || !auto_mapping_active)
  {
    return;
  }

  if ((point->distance_mm == 0U) ||
      (point->quality == 0U))
  {
    return;
  }

  robot_angle_cdeg = LidarPipeline_LidarToRobotAngleU16(point->angle_cdeg);
  clearance_mm = TestApp_ApplyForwardBodyOffset(robot_angle_cdeg, point->distance_mm);

  if (((point->flags & LIDAR_POINT_FLAG_SCAN_START) != 0U) &&
      (auto_mapping_observe_scan_starts_remaining > 0U))
  {
    auto_mapping_observe_scan_starts_remaining--;
  }

  if (TestApp_IsFrontLidarPoint(robot_angle_cdeg))
  {
    auto_mapping_front_last_seen_ms = HAL_GetTick();
    auto_mapping_front_last_robot_angle_cdeg = robot_angle_cdeg;
    auto_mapping_front_last_raw_angle_cdeg = point->angle_cdeg;
  }

  if (TestApp_IsFrontLidarPoint(robot_angle_cdeg) &&
      (clearance_mm < auto_mapping_front_min_mm))
  {
    auto_mapping_front_min_mm = clearance_mm;
    auto_mapping_front_blocking_angle_cdeg = robot_angle_cdeg;
  }

  if (TestApp_IsFrontLidarPoint(robot_angle_cdeg) &&
      (clearance_mm <= GetAutoFrontBlockDistanceMm()))
  {
    auto_mapping_front_blocked_until_ms = HAL_GetTick() + AUTO_MAPPING_FRONT_BLOCK_HOLD_MS;
    if (!angle_turn_active)
    {
      MotorControl_Stop();
    }
  }

  if (TestApp_IsLidarAngleNear(robot_angle_cdeg, AUTO_MAPPING_RIGHT_CENTER_CDEG, AUTO_MAPPING_SIDE_SECTOR_CDEG) &&
      (point->distance_mm < auto_mapping_right_min_mm))
  {
    auto_mapping_right_min_mm = point->distance_mm;
  }

  if (TestApp_IsLidarAngleNear(robot_angle_cdeg, AUTO_MAPPING_FRONT_RIGHT_CENTER_CDEG, AUTO_MAPPING_DIAGONAL_SECTOR_CDEG) &&
      (clearance_mm < auto_mapping_front_right_min_mm))
  {
    auto_mapping_front_right_min_mm = clearance_mm;
  }

  if ((TestApp_IsLidarAngleNear(robot_angle_cdeg, AUTO_MAPPING_RIGHT_CENTER_CDEG, AUTO_MAPPING_SIDE_SECTOR_CDEG) ||
       TestApp_IsLidarAngleNear(robot_angle_cdeg, AUTO_MAPPING_FRONT_RIGHT_CENTER_CDEG, AUTO_MAPPING_DIAGONAL_SECTOR_CDEG)) &&
      (point->distance_mm > auto_mapping_right_best_distance_mm))
  {
    auto_mapping_right_best_distance_mm = point->distance_mm;
    auto_mapping_right_best_angle_cdeg = robot_angle_cdeg;
  }

  if (TestApp_IsLidarAngleNear(robot_angle_cdeg, AUTO_MAPPING_LEFT_CENTER_CDEG, AUTO_MAPPING_SIDE_SECTOR_CDEG) &&
      (point->distance_mm < auto_mapping_left_min_mm))
  {
    auto_mapping_left_min_mm = point->distance_mm;
  }

  if ((TestApp_IsLidarAngleNear(robot_angle_cdeg, AUTO_MAPPING_LEFT_CENTER_CDEG, AUTO_MAPPING_SIDE_SECTOR_CDEG) ||
       TestApp_IsLidarAngleNear(robot_angle_cdeg, AUTO_MAPPING_FRONT_LEFT_CENTER_CDEG, AUTO_MAPPING_DIAGONAL_SECTOR_CDEG)) &&
      (point->distance_mm > auto_mapping_left_best_distance_mm))
  {
    auto_mapping_left_best_distance_mm = point->distance_mm;
    auto_mapping_left_best_angle_cdeg = robot_angle_cdeg;
  }

  if (!TestApp_IsFrontLidarPoint(robot_angle_cdeg) &&
      (point->distance_mm > auto_mapping_escape_best_distance_mm))
  {
    auto_mapping_escape_best_distance_mm = point->distance_mm;
    auto_mapping_escape_best_angle_cdeg = robot_angle_cdeg;
  }
}

static void TestApp_StartAutoObservation(const char *reason)
{
  char line[96];

  TestApp_ResetAutoMappingSectorMins();
  auto_mapping_observe_scan_starts_remaining = AUTO_MAPPING_OBSERVE_SCAN_STARTS;
  auto_mapping_observe_deadline_ms = HAL_GetTick() + AUTO_MAPPING_OBSERVE_TIMEOUT_MS;
  MotorControl_Stop();

  if (reason == NULL)
  {
    reason = "OBSERVE";
  }

  (void)snprintf(
      line,
      sizeof(line),
      "AUTO WALL OBSERVE %s scan_starts=%u\r\n",
      reason,
      (unsigned int)AUTO_MAPPING_OBSERVE_SCAN_STARTS);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_UpdateAutoMapping(uint32_t now_ms)
{
  MotorControlState_t motor_state = {0};
  uint16_t safe_mm;
  uint16_t side_open_mm;
  uint16_t right_open_mm;
  uint16_t front_right_open_mm;
  uint16_t right_wall_seen_mm;
  uint16_t drive_pwm;
  bool front_open;
  bool front_blocked;
  bool right_seen;
  bool front_right_seen;
  bool left_seen;
  bool right_open;
  bool front_right_open;
  bool left_open;
  bool right_turn_candidate;

  if (!auto_mapping_active)
  {
    TestApp_ResetAutoMappingSectorMins();
    return;
  }

  if (angle_turn_active)
  {
    TestApp_ResetAutoMappingSectorMins();
    return;
  }

  if ((int32_t)(now_ms - auto_mapping_resume_tick_ms) < 0L)
  {
    return;
  }

  if (auto_mapping_observe_after_resume)
  {
    auto_mapping_observe_after_resume = false;
    TestApp_StartAutoObservation("AFTER_TURN");
    return;
  }

  if (auto_mapping_observe_scan_starts_remaining > 0U)
  {
    if ((int32_t)(now_ms - auto_mapping_observe_deadline_ms) < 0L)
    {
      return;
    }
    auto_mapping_observe_scan_starts_remaining = 0U;
  }

  if ((now_ms - last_auto_mapping_control_tick_ms) < AUTO_MAPPING_CONTROL_INTERVAL_MS)
  {
    return;
  }
  last_auto_mapping_control_tick_ms = now_ms;

  safe_mm = GetAutoFrontBlockDistanceMm();
  side_open_mm = (uint16_t)(safe_mm + AUTO_MAPPING_OPEN_MARGIN_MM);
  right_open_mm = (uint16_t)(safe_mm + AUTO_MAPPING_RIGHT_OPEN_MARGIN_MM);
  if (right_open_mm < AUTO_MAPPING_MIN_RIGHT_OPEN_MM)
  {
    right_open_mm = AUTO_MAPPING_MIN_RIGHT_OPEN_MM;
  }
  front_right_open_mm = right_open_mm;
  if (front_right_open_mm < AUTO_MAPPING_MIN_FRONT_RIGHT_OPEN_MM)
  {
    front_right_open_mm = AUTO_MAPPING_MIN_FRONT_RIGHT_OPEN_MM;
  }
  right_wall_seen_mm = (uint16_t)(safe_mm + AUTO_MAPPING_RIGHT_WALL_MARGIN_MM);
  front_blocked = TestApp_IsAutoFrontBlocked(now_ms);
  front_open = !front_blocked &&
      ((auto_mapping_front_min_mm == UINT16_MAX) || (auto_mapping_front_min_mm > safe_mm));
  right_seen = (auto_mapping_right_min_mm != UINT16_MAX);
  front_right_seen = (auto_mapping_front_right_min_mm != UINT16_MAX);
  left_seen = (auto_mapping_left_min_mm != UINT16_MAX);
  right_open = right_seen && (auto_mapping_right_min_mm > right_open_mm);
  front_right_open = front_right_seen && (auto_mapping_front_right_min_mm > front_right_open_mm);
  left_open = left_seen && (auto_mapping_left_min_mm > side_open_mm);

  if (right_seen && (auto_mapping_right_min_mm <= right_wall_seen_mm))
  {
    auto_mapping_right_wall_seen = true;
    auto_mapping_right_branch_confirm_count = 0U;
  }

  right_turn_candidate = right_open &&
      front_right_open &&
      (front_blocked || (front_open && auto_mapping_right_wall_seen)) &&
      ((int32_t)(now_ms - auto_mapping_ignore_right_until_ms) >= 0L);

  if (right_turn_candidate)
  {
    if (auto_mapping_right_branch_confirm_count < AUTO_MAPPING_RIGHT_BRANCH_CONFIRM_COUNT)
    {
      auto_mapping_right_branch_confirm_count++;
    }
  }
  else if (!right_open || !front_right_open)
  {
    auto_mapping_right_branch_confirm_count = 0U;
  }

  if (right_turn_candidate &&
      (auto_mapping_right_branch_confirm_count >= AUTO_MAPPING_RIGHT_BRANCH_CONFIRM_COUNT))
  {
    MotorControl_Stop();
    TestApp_AutoMappingStartTurn(1, AUTO_MAPPING_GRID_TURN_DEG, "RIGHT_OPEN", now_ms);
  }
  else if (front_open)
  {
    drive_pwm = ClampPwmPermille(GetDrivePwmPermille(), AUTO_MAPPING_MAX_DRIVE_PWM);
    if (!MotorControl_GetState(&motor_state) ||
        (motor_state.mode != 1U) ||
        PwmValueChanged(motor_state.duty_permille, drive_pwm))
    {
      MotorControl_SetForward(drive_pwm);
    }
  }
  else if (left_open)
  {
    MotorControl_Stop();
    TestApp_AutoMappingStartTurn(-1, AUTO_MAPPING_GRID_TURN_DEG, "LEFT_OPEN", now_ms);
  }
  else
  {
    MotorControl_Stop();
    TestApp_AutoMappingStartTurn(1, AUTO_MAPPING_U_TURN_DEG, "DEAD_END", now_ms);
  }

  TestApp_ResetAutoMappingSectorMins();
}

static void TestApp_ResetAutoMappingSectorMins(void)
{
  auto_mapping_front_min_mm = UINT16_MAX;
  auto_mapping_front_blocking_angle_cdeg = 0U;
  auto_mapping_right_min_mm = UINT16_MAX;
  auto_mapping_front_right_min_mm = UINT16_MAX;
  auto_mapping_left_min_mm = UINT16_MAX;
  auto_mapping_right_best_distance_mm = 0U;
  auto_mapping_right_best_angle_cdeg = AUTO_MAPPING_RIGHT_CENTER_CDEG;
  auto_mapping_left_best_distance_mm = 0U;
  auto_mapping_left_best_angle_cdeg = AUTO_MAPPING_LEFT_CENTER_CDEG;
  auto_mapping_escape_best_distance_mm = 0U;
  auto_mapping_escape_best_angle_cdeg = AUTO_MAPPING_LEFT_CENTER_CDEG;
}

static bool TestApp_IsAutoFrontBlocked(uint32_t now_ms)
{
  return ((auto_mapping_front_blocked_until_ms != 0U) &&
          ((int32_t)(now_ms - auto_mapping_front_blocked_until_ms) < 0L));
}

static int32_t TestApp_SignedHeadingErrorCdeg(int32_t target_cdeg, int32_t current_cdeg)
{
  int32_t error = NormalizeHeadingCdeg(target_cdeg) - NormalizeHeadingCdeg(current_cdeg);

  if (error > 18000L)
  {
    error -= 36000L;
  }
  else if (error < -18000L)
  {
    error += 36000L;
  }

  return error;
}

static int32_t TestApp_SnapHeadingToMazeAxis(int32_t heading_cdeg)
{
  int32_t normalized = NormalizeHeadingCdeg(heading_cdeg);

  return NormalizeHeadingCdeg(((normalized + 4500L) / 9000L) * 9000L);
}

static int32_t TestApp_GetAutoTurnTargetHeading(int8_t direction, uint16_t requested_degrees)
{
  int32_t base_heading = TestApp_SnapHeadingToMazeAxis(mapping_pose.heading_cdeg);
  int32_t step_cdeg = (requested_degrees >= 135U) ? 18000L : 9000L;

  if (direction < 0)
  {
    return NormalizeHeadingCdeg(base_heading + step_cdeg);
  }

  if (direction > 0)
  {
    return NormalizeHeadingCdeg(base_heading - step_cdeg);
  }

  return base_heading;
}

static void TestApp_AutoMappingStartTurn(int8_t direction, uint16_t degrees, const char *reason, uint32_t now_ms)
{
  char line[224];
  int32_t target_heading_cdeg;
  int32_t snapped_error_cdeg;
  uint16_t snapped_degrees;

  auto_mapping_avoid_count++;
  auto_mapping_resume_tick_ms = now_ms + AUTO_MAPPING_TURN_SETTLE_MS;
  auto_mapping_ignore_right_until_ms = auto_mapping_resume_tick_ms + AUTO_MAPPING_POST_TURN_DRIVE_MS;
  auto_mapping_right_branch_confirm_count = 0U;
  auto_mapping_right_wall_seen = false;
  if (degrees >= 135U)
  {
    degrees = AUTO_MAPPING_U_TURN_DEG;
    auto_mapping_last_turn_target_angle_cdeg = AUTO_MAPPING_BACK_CENTER_CDEG;
  }
  else
  {
    degrees = AUTO_MAPPING_GRID_TURN_DEG;
    auto_mapping_last_turn_target_angle_cdeg =
        (direction < 0) ? AUTO_MAPPING_LEFT_CENTER_CDEG : AUTO_MAPPING_RIGHT_CENTER_CDEG;
  }
  target_heading_cdeg = TestApp_GetAutoTurnTargetHeading(direction, degrees);
  snapped_error_cdeg = TestApp_SignedHeadingErrorCdeg(target_heading_cdeg, mapping_pose.heading_cdeg);
  snapped_degrees = (uint16_t)((AppAbs32(snapped_error_cdeg) + 50L) / 100L);
  TestApp_StartHeadingTurn(target_heading_cdeg, 0U, reason);

  if (reason == NULL)
  {
    reason = "TURN";
  }

  (void)snprintf(
      line,
      sizeof(line),
      "AUTO WALL %s dir=%c req=%u snap=%u head=%ld target=%u front=%u fraw=%u fang=%u right=%u fr=%u left=%u wall=%u conf=%u count=%lu\r\n",
      reason,
      (direction < 0) ? 'L' : 'R',
      (unsigned int)degrees,
      (unsigned int)snapped_degrees,
      (long)target_heading_cdeg,
      (unsigned int)auto_mapping_last_turn_target_angle_cdeg,
      (unsigned int)auto_mapping_front_min_mm,
      (unsigned int)auto_mapping_front_last_raw_angle_cdeg,
      (unsigned int)auto_mapping_front_last_robot_angle_cdeg,
      (unsigned int)auto_mapping_right_min_mm,
      (unsigned int)auto_mapping_front_right_min_mm,
      (unsigned int)auto_mapping_left_min_mm,
      (unsigned int)auto_mapping_right_wall_seen,
      (unsigned int)auto_mapping_right_branch_confirm_count,
      (unsigned long)auto_mapping_avoid_count);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_StreamMap(void)
{
  char row_text[MAPPING_GRID_WIDTH_CELLS + 1U];
  char line[MAPPING_GRID_WIDTH_CELLS + 48U];
  uint32_t now = HAL_GetTick();
  uint32_t revision;
  uint8_t rows_sent = 0U;

  if (!mapping_active)
  {
    return;
  }

  if ((now - last_map_stat_tx_tick_ms) >= MAP_STAT_TX_INTERVAL_MS)
  {
    last_map_stat_tx_tick_ms = now;
    TestApp_SendMapStat();
  }

  if ((now - last_map_row_tx_tick_ms) < MAP_ROW_TX_INTERVAL_MS)
  {
    return;
  }

  last_map_row_tx_tick_ms = now;
  revision = MappingGrid_GetRevision();

  while (rows_sent < MAP_ROWS_PER_TX_BURST)
  {
    if (MappingGrid_FormatRow(next_map_tx_row, row_text, sizeof(row_text)))
    {
      (void)snprintf(
          line,
          sizeof(line),
          "MAP ROW y=%u rev=%lu data=%s\r\n",
          (unsigned int)next_map_tx_row,
          (unsigned long)revision,
          row_text);
      (void)BluetoothControl_SendText(line);
    }

    next_map_tx_row++;
    if (next_map_tx_row >= MAPPING_GRID_HEIGHT_CELLS)
    {
      next_map_tx_row = 0U;
    }
    rows_sent++;
  }
}

static void TestApp_StreamPose(void)
{
  MotorControlState_t motor_state = {0};
  char line[128];
  uint32_t now = HAL_GetTick();

  if (!mapping_active)
  {
    return;
  }

  if ((now - last_pose_tx_tick_ms) < POSE_TX_INTERVAL_MS)
  {
    return;
  }
  last_pose_tx_tick_ms = now;

  (void)MotorControl_GetState(&motor_state);
  (void)snprintf(
      line,
      sizeof(line),
      "POSE %ld,%ld,%ld mpu=%u gz=%ld ld=%d rd=%d mode=%u fw=%u\r\n",
      (long)mapping_pose.x_mm,
      (long)mapping_pose.y_mm,
      (long)mapping_pose.heading_cdeg,
      (unsigned int)mpu_state.ready,
      (long)gyro_z_corrected_dps_x100,
      (int)encoder_test.left_delta,
      (int)encoder_test.right_delta,
      (unsigned int)motor_state.mode,
      (unsigned int)motor_state.forward_active);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_RequestFullMapStream(void)
{
  next_map_tx_row = 0U;
  last_map_row_tx_tick_ms = 0U;
  TestApp_SendMapHeader(mapping_active ? "SNAP" : "IDLE");
  TestApp_SendMapStat();
}

static void TestApp_SendMapHeader(const char *state)
{
  char line[96];

  if (state == NULL)
  {
    state = "INFO";
  }

  (void)snprintf(
      line,
      sizeof(line),
      "MAP %s w=%u h=%u cell=%umm rev=%lu\r\n",
      state,
      (unsigned int)MAPPING_GRID_WIDTH_CELLS,
      (unsigned int)MAPPING_GRID_HEIGHT_CELLS,
      (unsigned int)MAPPING_GRID_CELL_SIZE_MM,
      (unsigned long)MappingGrid_GetRevision());
  (void)BluetoothControl_SendText(line);
}

static void TestApp_SendMapStat(void)
{
  MappingGridStats_t stats;
  BluetoothControlState_t bt_state = {0};
  char line[224];

  if (!MappingGrid_GetStats(&stats))
  {
    return;
  }

  (void)BluetoothControl_GetState(&bt_state);

  (void)snprintf(
      line,
      sizeof(line),
      "MAP STAT active=%u rev=%lu inserted=%lu rejected=%lu unknown=%u free=%u occupied=%u clipped=%lu drops=%lu txdrop=%lu qmin=%u bias=%u pose=%ld,%ld,%ld\r\n",
      mapping_active ? 1U : 0U,
      (unsigned long)stats.revision,
      (unsigned long)stats.inserted_points,
      (unsigned long)stats.rejected_points,
      (unsigned int)stats.unknown_cells,
      (unsigned int)stats.free_cells,
      (unsigned int)stats.occupied_cells,
      (unsigned long)stats.clipped_rays,
      (unsigned long)LidarPipeline_GetPointQueueDrops(),
      (unsigned long)bt_state.tx_drops,
      (unsigned int)LidarPipeline_GetMinPointQuality(),
      (unsigned int)LidarPipeline_GetDistanceBiasMm(),
      (long)mapping_pose.x_mm,
      (long)mapping_pose.y_mm,
      (long)mapping_pose.heading_cdeg);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_SendMpuState(void)
{
  char line[192];

  (void)snprintf(
      line,
      sizeof(line),
      "MPU STATE ready=%u who=%02X raw=%d gz=%ld bias=%ld corr=%ld heading=%ld map=%u cal=%u\r\n",
      (unsigned int)mpu_state.ready,
      (unsigned int)mpu_state.who_am_i,
      (int)mpu_state.gyro_z_raw,
      (long)mpu_state.gyro_z_dps_x100,
      (long)gyro_z_bias_dps_x100,
      (long)gyro_z_corrected_dps_x100,
      (long)mapping_pose.heading_cdeg,
      (unsigned int)mapping_active,
      (unsigned int)gyro_calibration_active);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_ResetMapNorth(void)
{
  char line[128];
  uint32_t now = HAL_GetTick();
  int32_t old_heading_cdeg = NormalizeHeadingCdeg(mapping_pose.heading_cdeg);

  mapping_pose.heading_cdeg = MAPPING_START_HEADING_CDEG;
  odom_debug_heading_cdeg = 0L;
  gyro_z_corrected_dps_x100 = 0L;
  mapping_travel_residual_x1000 = 0L;
  last_mapping_pose_tick_ms = now;

  MappingGrid_SetPose(&mapping_pose);
  TestApp_ResetPoseHistory();
  TestApp_RecordPoseHistory(now);

  (void)snprintf(
      line,
      sizeof(line),
      "DIR RESET old=%ld new=%ld map=%u mpu=%u bias=%ld\r\n",
      (long)old_heading_cdeg,
      (long)mapping_pose.heading_cdeg,
      (unsigned int)mapping_active,
      (unsigned int)mpu_state.ready,
      (long)gyro_z_bias_dps_x100);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_SendDirState(void)
{
  char line[128];
  int32_t heading_cdeg = NormalizeHeadingCdeg(mapping_pose.heading_cdeg);

  (void)snprintf(
      line,
      sizeof(line),
      "DIR heading=%ld deg=%ld.%02ld map=%u mpu=%u bias=%ld corr=%ld\r\n",
      (long)heading_cdeg,
      (long)(heading_cdeg / 100L),
      (long)(heading_cdeg % 100L),
      (unsigned int)mapping_active,
      (unsigned int)mpu_state.ready,
      (long)gyro_z_bias_dps_x100,
      (long)gyro_z_corrected_dps_x100);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_UpdateMotorSpeedFromAdc(void)
{
  MotorControlState_t motor_state = {0};
  uint16_t target_pwm;

  if (auto_mapping_active || angle_turn_active || SlamNav_IsActive())
  {
    return;
  }

  if (!MotorControl_GetState(&motor_state))
  {
    return;
  }

  if (motor_state.mode == 1U)
  {
    target_pwm = GetDrivePwmPermille();
    if (PwmValueChanged(motor_state.duty_permille, target_pwm))
    {
      MotorControl_SetForward(target_pwm);
    }
  }
  else if (motor_state.mode == 2U)
  {
    target_pwm = GetTurnPwmPermille();
    if (PwmValueChanged(motor_state.duty_permille, target_pwm))
    {
      MotorControl_SetTurnLeft(target_pwm);
    }
  }
  else if (motor_state.mode == 3U)
  {
    target_pwm = GetTurnPwmPermille();
    if (PwmValueChanged(motor_state.duty_permille, target_pwm))
    {
      MotorControl_SetTurnRight(target_pwm);
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  TestApp_InitPeripherals();
  app_ready = true;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(senseTask, StartSenseTask, osPriorityAboveNormal, 0, 512);
  senseTaskHandle = osThreadCreate(osThread(senseTask), NULL);

  osThreadDef(uiTask, StartUiTask, osPriorityBelowNormal, 0, 512);
  uiTaskHandle = osThreadCreate(osThread(uiTask), NULL);

  osThreadDef(btTask, StartBtTask, osPriorityHigh, 0, 384);
  btTaskHandle = osThreadCreate(osThread(btTask), NULL);

  osThreadDef(lidarAppTask, StartLidarAppTask, osPriorityAboveNormal, 0, 384);
  lidarAppTaskHandle = osThreadCreate(osThread(lidarAppTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch)
{
  uint8_t byte = (uint8_t)ch;

  (void)HAL_UART_Transmit(&huart2, &byte, 1U, 10U);
  return ch;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  (void)argument;

  vTaskDelete(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSenseTask */
/**
  * @brief  Function implementing the senseTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSenseTask */
void StartSenseTask(void const * argument)
{
  /* USER CODE BEGIN StartSenseTask */
  (void)argument;

  for(;;)
  {
    if (app_ready)
    {
      MotorControl_UpdateButtons();
      TestApp_UpdateSensors();
    }
    osDelay(2);
  }
  /* USER CODE END StartSenseTask */
}

/* USER CODE BEGIN Header_StartUiTask */
/**
  * @brief  Function implementing the uiTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUiTask */
void StartUiTask(void const * argument)
{
  /* USER CODE BEGIN StartUiTask */
  (void)argument;

  for(;;)
  {
    if (app_ready)
    {
      TestApp_PollButtons();
      TestApp_UpdateDisplay();
    }
    osDelay(20);
  }
  /* USER CODE END StartUiTask */
}

/* USER CODE BEGIN Header_StartBtTask */
/**
  * @brief  Function implementing the btTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBtTask */
void StartBtTask(void const * argument)
{
  /* USER CODE BEGIN StartBtTask */
  (void)argument;

  for(;;)
  {
    if (app_ready)
    {
      BluetoothControl_Update();
      TestApp_HandleBluetoothCommands();
      TestApp_StreamMap();
      TestApp_StreamPose();
      TestApp_StreamOdomDebug();
    }
    osDelay(20);
  }
  /* USER CODE END StartBtTask */
}

/* USER CODE BEGIN Header_StartLidarAppTask */
/**
  * @brief  Function implementing the lidarAppTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLidarAppTask */
void StartLidarAppTask(void const * argument)
{
  /* USER CODE BEGIN StartLidarAppTask */
  (void)argument;

  for(;;)
  {
    if (app_ready)
    {
      TestApp_ProcessLidarPoints();
    }
    osDelay(5);
  }
  /* USER CODE END StartLidarAppTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
