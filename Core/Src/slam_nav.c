#include "slam_nav.h"

#include <stdio.h>
#include <string.h>

#include "astar_planner.h"
#include "bluetooth_control.h"
#include "FreeRTOS.h"
#include "main.h"
#include "mapping_grid.h"
#include "motor_control.h"
#include "queue.h"
#include "task.h"

#define SLAM_NAV_TASK_STACK_WORDS       640U
#define SLAM_NAV_TASK_PRIORITY          (tskIDLE_PRIORITY + 2U)
#define SLAM_NAV_COMMAND_QUEUE_LENGTH   4U
#define SLAM_NAV_TASK_PERIOD_MS         20U
#define SLAM_NAV_DEFAULT_DRIVE_PWM      340U
#define SLAM_NAV_DEFAULT_TURN_PWM       330U
#define SLAM_NAV_DEFAULT_SAFE_MM        350U
#define SLAM_NAV_MAX_PWM                1000U
#define SLAM_NAV_MAX_DRIVE_PWM          420U
#define SLAM_NAV_MAX_TURN_PWM           390U
#define SLAM_NAV_MIN_SAFE_MM            50U
#define SLAM_NAV_MAX_SAFE_MM            2000U
#define SLAM_NAV_MIN_DRIVE_PWM          300U
#define SLAM_NAV_MIN_TURN_PWM           300U
#define SLAM_NAV_FRONT_SECTOR_CDEG      1500U
#define SLAM_NAV_SIDE_SECTOR_CDEG       3000U
#define SLAM_NAV_DIAGONAL_SECTOR_CDEG   1800U
#define SLAM_NAV_RIGHT_CENTER_CDEG      27000U
#define SLAM_NAV_BACK_CENTER_CDEG       18000U
#define SLAM_NAV_LEFT_CENTER_CDEG       9000U
#define SLAM_NAV_FRONT_RIGHT_CENTER_CDEG 31500U
#define SLAM_NAV_FRONT_LEFT_CENTER_CDEG 4500U
#define SLAM_NAV_FRONT_BODY_OFFSET_MM  200U
#define SLAM_NAV_FRONT_DIAGONAL_BODY_OFFSET_MM 140U
#define SLAM_NAV_TURN_TOL_CDEG          1200L
#define SLAM_NAV_TURN_SETTLE_MS         280U
#define SLAM_NAV_DRIVE_HEADING_TOL_CDEG 2600L
#define SLAM_NAV_HEADING_HOLD_DEADBAND_CDEG 150L
#define SLAM_NAV_HEADING_HOLD_CDEG_PER_PWM  40L
#define SLAM_NAV_HEADING_HOLD_MAX_STEER     70L
#define SLAM_NAV_TARGET_RADIUS_MM       120L
#define SLAM_NAV_ROBOT_FREE_RADIUS      1U
#define SLAM_NAV_HEARTBEAT_INTERVAL_MS  500U
#define SLAM_NAV_STATUS_INTERVAL_MS     300U
#define SLAM_NAV_PATH_TX_INTERVAL_MS    1000U
#define SLAM_NAV_PATH_CHUNK_CELLS       10U
#define SLAM_NAV_TURN_TIMEOUT_MS        6000U
#define SLAM_NAV_SECTOR_STALE_MS        1000U
#define SLAM_NAV_PATH_VALIDATE_INTERVAL_MS 150U
#define SLAM_NAV_CENTER_GOAL_X_MM       0L
#define SLAM_NAV_CENTER_GOAL_Y_MM       0L
#define SLAM_NAV_MOTION_STEP_MM         700U
#define SLAM_NAV_MOTION_STEP_CELLS      ((uint16_t)((SLAM_NAV_MOTION_STEP_MM + (MAPPING_GRID_CELL_SIZE_MM / 2U)) / MAPPING_GRID_CELL_SIZE_MM))

typedef enum
{
  SLAM_NAV_COMMAND_START_EXPLORE = 0,
  SLAM_NAV_COMMAND_START_RETURN,
  SLAM_NAV_COMMAND_STOP
} SlamNavCommand_t;

typedef enum
{
  SLAM_NAV_MODE_EXPLORE = 0,
  SLAM_NAV_MODE_RETURN
} SlamNavMode_t;

typedef struct
{
  SlamNavCommand_t command;
  int32_t goal_x_mm;
  int32_t goal_y_mm;
} SlamNavCommandMessage_t;

typedef struct
{
  uint16_t front_mm;
  uint16_t right_mm;
  uint16_t left_mm;
  uint16_t back_mm;
  uint16_t front_right_mm;
  uint16_t front_left_mm;
  uint32_t last_tick_ms;
} SlamNavSectorStats_t;

static StaticTask_t s_nav_task_struct;
static StackType_t s_nav_task_stack[SLAM_NAV_TASK_STACK_WORDS];
static TaskHandle_t s_nav_task_handle;

static StaticQueue_t s_command_queue_struct;
static uint8_t s_command_queue_storage[SLAM_NAV_COMMAND_QUEUE_LENGTH * sizeof(SlamNavCommandMessage_t)];
static QueueHandle_t s_command_queue;

static MappingGridSnapshot_t s_snapshot;
static AstarPlannerPath_t s_path;

static bool s_initialized;
static bool s_active;
static SlamNavMode_t s_mode = SLAM_NAV_MODE_EXPLORE;
static SlamNavState_t s_state = SLAM_NAV_STATE_IDLE;
static uint16_t s_drive_pwm_permille = SLAM_NAV_DEFAULT_DRIVE_PWM;
static uint16_t s_turn_pwm_permille = SLAM_NAV_DEFAULT_TURN_PWM;
static uint16_t s_safe_distance_mm = SLAM_NAV_DEFAULT_SAFE_MM;
static uint16_t s_front_min_distance_mm;
static uint16_t s_path_index;
static uint32_t s_path_revision;
static uint32_t s_last_path_validate_tick_ms;
static AstarPlannerCell_t s_current_target_cell;
static int32_t s_target_x_mm;
static int32_t s_target_y_mm;
static int32_t s_target_heading_cdeg;
static uint32_t s_plan_seq;
static uint32_t s_last_heartbeat_tick_ms;
static uint32_t s_last_status_tick_ms;
static uint32_t s_last_path_tx_tick_ms;
static uint32_t s_state_enter_tick_ms;
static uint32_t s_turn_settle_until_ms;
static bool s_last_path_tx_valid;
static AstarPlannerCell_t s_last_path_tx_target;
static uint16_t s_last_path_tx_length;
static int32_t s_return_goal_x_mm;
static int32_t s_return_goal_y_mm;
static SlamNavSectorStats_t s_sector_current;
static SlamNavSectorStats_t s_sector_last;
static bool s_escape_replan_after_turn;

static void SlamNav_Task(void *argument);
static void SlamNav_HandleCommand(const SlamNavCommandMessage_t *message);
static void SlamNav_Update(void);
static void SlamNav_StartInternal(SlamNavMode_t mode, int32_t goal_x_mm, int32_t goal_y_mm);
static void SlamNav_StopInternal(const char *reason, bool send_status);
static void SlamNav_UpdatePlan(void);
static void SlamNav_UpdateTurn(void);
static void SlamNav_UpdateDrive(void);
static bool SlamNav_GetPoseAndCell(MappingGridPose_t *out_pose, uint8_t *out_x, uint8_t *out_y);
static bool SlamNav_SetTargetFromPath(uint16_t path_index);
static uint16_t SlamNav_SelectNextPathIndex(uint16_t from_path_index);
static bool SlamNav_PathStillTraversable(uint32_t now);
static bool SlamNav_IsFrontAngle(uint16_t angle_cdeg);
static bool SlamNav_IsAngleNear(uint16_t angle_cdeg, uint16_t center_cdeg, uint16_t half_width_cdeg);
static int32_t SlamNav_NormalizeHeadingCdeg(int32_t heading_cdeg);
static int32_t SlamNav_SignedHeadingErrorCdeg(int32_t target_cdeg, int32_t current_cdeg);
static int32_t SlamNav_Abs32(int32_t value);
static uint16_t SlamNav_ClampPwm(uint16_t value);
static void SlamNav_ResetSectorStats(SlamNavSectorStats_t *stats);
static void SlamNav_UpdateSectorMin(uint16_t *value, uint16_t distance_mm);
static void SlamNav_UpdateSectorStats(uint16_t robot_angle_cdeg, uint16_t distance_mm);
static SlamNavSectorStats_t SlamNav_GetSectorSnapshot(uint32_t now);
static uint16_t SlamNav_ApplyForwardBodyOffset(uint16_t robot_angle_cdeg, uint16_t distance_mm);
static uint16_t SlamNav_ActivePwm(uint16_t configured_pwm, uint16_t fallback_pwm, uint16_t minimum_pwm);
static uint16_t SlamNav_ClampConfiguredPwm(uint16_t value, uint16_t maximum);
static int32_t SlamNav_HeadingHoldSteer(int32_t error_cdeg);
static int32_t SlamNav_CellHeadingCdeg(const AstarPlannerCell_t *from, const AstarPlannerCell_t *to);
static void SlamNav_SendStatus(const char *state, const char *reason);
static void SlamNav_SendHeartbeatIfDue(void);
static void SlamNav_SendPath(void);

bool SlamNav_Init(void)
{
  if (s_initialized)
  {
    return true;
  }

  s_command_queue = xQueueCreateStatic(
      SLAM_NAV_COMMAND_QUEUE_LENGTH,
      sizeof(SlamNavCommandMessage_t),
      s_command_queue_storage,
      &s_command_queue_struct);
  configASSERT(s_command_queue != NULL);

  s_nav_task_handle = xTaskCreateStatic(
      SlamNav_Task,
      "slamNav",
      SLAM_NAV_TASK_STACK_WORDS,
      NULL,
      SLAM_NAV_TASK_PRIORITY,
      s_nav_task_stack,
      &s_nav_task_struct);
  configASSERT(s_nav_task_handle != NULL);

  SlamNav_ResetSectorStats(&s_sector_current);
  SlamNav_ResetSectorStats(&s_sector_last);
  s_initialized = true;
  return true;
}

void SlamNav_StartExplore(void)
{
  SlamNavCommandMessage_t message;

  if (!s_initialized)
  {
    (void)SlamNav_Init();
  }

  message.command = SLAM_NAV_COMMAND_START_EXPLORE;
  message.goal_x_mm = SLAM_NAV_CENTER_GOAL_X_MM;
  message.goal_y_mm = SLAM_NAV_CENTER_GOAL_Y_MM;

  if (s_command_queue != NULL)
  {
    (void)xQueueSend(s_command_queue, &message, 0U);
  }
}

void SlamNav_StartReturnTo(int32_t goal_x_mm, int32_t goal_y_mm)
{
  SlamNavCommandMessage_t message;

  if (!s_initialized)
  {
    (void)SlamNav_Init();
  }

  message.command = SLAM_NAV_COMMAND_START_RETURN;
  message.goal_x_mm = goal_x_mm;
  message.goal_y_mm = goal_y_mm;

  if (s_command_queue != NULL)
  {
    (void)xQueueSend(s_command_queue, &message, 0U);
  }
}

void SlamNav_Stop(void)
{
  SlamNav_StopInternal("STOP", false);
}

bool SlamNav_IsActive(void)
{
  bool active;

  taskENTER_CRITICAL();
  active = s_active;
  taskEXIT_CRITICAL();
  return active;
}

SlamNavState_t SlamNav_GetState(void)
{
  SlamNavState_t state;

  taskENTER_CRITICAL();
  state = s_state;
  taskEXIT_CRITICAL();
  return state;
}

void SlamNav_SetControlConfig(uint16_t drive_pwm_permille,
                              uint16_t turn_pwm_permille,
                              uint16_t safe_distance_mm)
{
  taskENTER_CRITICAL();
  s_drive_pwm_permille = SlamNav_ClampConfiguredPwm(drive_pwm_permille, SLAM_NAV_MAX_DRIVE_PWM);
  s_turn_pwm_permille = SlamNav_ClampConfiguredPwm(turn_pwm_permille, SLAM_NAV_MAX_TURN_PWM);
  if (safe_distance_mm < SLAM_NAV_MIN_SAFE_MM)
  {
    s_safe_distance_mm = SLAM_NAV_MIN_SAFE_MM;
  }
  else if (safe_distance_mm > SLAM_NAV_MAX_SAFE_MM)
  {
    s_safe_distance_mm = SLAM_NAV_MAX_SAFE_MM;
  }
  else
  {
    s_safe_distance_mm = safe_distance_mm;
  }
  taskEXIT_CRITICAL();
}

void SlamNav_ObserveLidarPoint(const LidarPoint_t *point)
{
  uint16_t robot_angle_cdeg;
  uint16_t clearance_mm;

  if ((point == NULL) ||
      !s_active ||
      (point->quality == 0U) ||
      (point->distance_mm == 0U))
  {
    return;
  }

  robot_angle_cdeg = LidarPipeline_LidarToRobotAngleU16(point->angle_cdeg);
  if ((point->flags & LIDAR_POINT_FLAG_SCAN_START) != 0U)
  {
    taskENTER_CRITICAL();
    if (s_sector_current.last_tick_ms != 0U)
    {
      s_sector_last = s_sector_current;
    }
    SlamNav_ResetSectorStats(&s_sector_current);
    s_front_min_distance_mm = 0U;
    taskEXIT_CRITICAL();
  }

  clearance_mm = SlamNav_ApplyForwardBodyOffset(robot_angle_cdeg, point->distance_mm);
  if (clearance_mm == 0U)
  {
    return;
  }

  SlamNav_UpdateSectorStats(robot_angle_cdeg, clearance_mm);

  if (!SlamNav_IsFrontAngle(robot_angle_cdeg))
  {
    return;
  }

  taskENTER_CRITICAL();
  if ((s_front_min_distance_mm == 0U) || (clearance_mm < s_front_min_distance_mm))
  {
    s_front_min_distance_mm = clearance_mm;
  }
  taskEXIT_CRITICAL();
}

const char *SlamNav_StateName(SlamNavState_t state)
{
  switch (state)
  {
    case SLAM_NAV_STATE_IDLE:    return "IDLE";
    case SLAM_NAV_STATE_PLAN:    return "PLAN";
    case SLAM_NAV_STATE_TURN:    return "TURN";
    case SLAM_NAV_STATE_DRIVE:   return "DRIVE";
    case SLAM_NAV_STATE_REPLAN:  return "REPLAN";
    case SLAM_NAV_STATE_DONE:    return "DONE";
    case SLAM_NAV_STATE_NO_PATH: return "NO_PATH";
    default:                     return "UNKNOWN";
  }
}

static void SlamNav_Task(void *argument)
{
  SlamNavCommandMessage_t message;

  (void)argument;

  for (;;)
  {
    while ((s_command_queue != NULL) &&
           (xQueueReceive(s_command_queue, &message, 0U) == pdPASS))
    {
      SlamNav_HandleCommand(&message);
    }

    SlamNav_Update();
    vTaskDelay(pdMS_TO_TICKS(SLAM_NAV_TASK_PERIOD_MS));
  }
}

static void SlamNav_HandleCommand(const SlamNavCommandMessage_t *message)
{
  if (message == NULL)
  {
    return;
  }

  if (message->command == SLAM_NAV_COMMAND_START_EXPLORE)
  {
    SlamNav_StartInternal(SLAM_NAV_MODE_EXPLORE, 0L, 0L);
  }
  else if (message->command == SLAM_NAV_COMMAND_START_RETURN)
  {
    SlamNav_StartInternal(SLAM_NAV_MODE_RETURN, message->goal_x_mm, message->goal_y_mm);
  }
  else
  {
    SlamNav_StopInternal("STOP", true);
  }
}

static void SlamNav_Update(void)
{
  if (!s_active)
  {
    return;
  }

  SlamNav_SendHeartbeatIfDue();

  switch (s_state)
  {
    case SLAM_NAV_STATE_PLAN:
    case SLAM_NAV_STATE_REPLAN:
      SlamNav_UpdatePlan();
      break;
    case SLAM_NAV_STATE_TURN:
      SlamNav_UpdateTurn();
      break;
    case SLAM_NAV_STATE_DRIVE:
      SlamNav_UpdateDrive();
      break;
    case SLAM_NAV_STATE_IDLE:
    case SLAM_NAV_STATE_DONE:
    case SLAM_NAV_STATE_NO_PATH:
    default:
      break;
  }
}

static void SlamNav_StartInternal(SlamNavMode_t mode, int32_t goal_x_mm, int32_t goal_y_mm)
{
  taskENTER_CRITICAL();
  s_active = true;
  s_mode = mode;
  s_state = SLAM_NAV_STATE_PLAN;
  s_state_enter_tick_ms = HAL_GetTick();
  s_front_min_distance_mm = 0U;
  s_path_index = 0U;
  s_path_revision = 0U;
  s_last_path_validate_tick_ms = 0U;
  s_plan_seq = 0U;
  s_turn_settle_until_ms = 0U;
  s_last_heartbeat_tick_ms = 0U;
  s_last_status_tick_ms = 0U;
  s_last_path_tx_tick_ms = 0U;
  s_last_path_tx_valid = false;
  s_last_path_tx_length = 0U;
  s_return_goal_x_mm = (mode == SLAM_NAV_MODE_EXPLORE) ? SLAM_NAV_CENTER_GOAL_X_MM : goal_x_mm;
  s_return_goal_y_mm = (mode == SLAM_NAV_MODE_EXPLORE) ? SLAM_NAV_CENTER_GOAL_Y_MM : goal_y_mm;
  s_escape_replan_after_turn = false;
  SlamNav_ResetSectorStats(&s_sector_current);
  SlamNav_ResetSectorStats(&s_sector_last);
  taskEXIT_CRITICAL();

  MotorControl_Stop();
  SlamNav_SendStatus("START", (mode == SLAM_NAV_MODE_RETURN) ? "RETURN" : "CENTER");
}

static void SlamNav_StopInternal(const char *reason, bool send_status)
{
  bool was_active;

  taskENTER_CRITICAL();
  was_active = s_active;
  s_active = false;
  s_mode = SLAM_NAV_MODE_EXPLORE;
  s_state = SLAM_NAV_STATE_IDLE;
  s_state_enter_tick_ms = HAL_GetTick();
  s_front_min_distance_mm = 0U;
  s_path_index = 0U;
  s_path_revision = 0U;
  s_last_path_validate_tick_ms = 0U;
  s_last_path_tx_valid = false;
  s_turn_settle_until_ms = 0U;
  s_escape_replan_after_turn = false;
  SlamNav_ResetSectorStats(&s_sector_current);
  SlamNav_ResetSectorStats(&s_sector_last);
  taskEXIT_CRITICAL();

  if (was_active)
  {
    MotorControl_Stop();
  }

  if (send_status && was_active)
  {
    SlamNav_SendStatus("STOP", reason);
  }
}

static void SlamNav_UpdatePlan(void)
{
  MappingGridPose_t pose;
  uint8_t start_x;
  uint8_t start_y;
  uint8_t goal_x = 0U;
  uint8_t goal_y = 0U;
  AstarPlannerStatus_t status;
  SlamNavMode_t mode;
  int32_t return_goal_x_mm;
  int32_t return_goal_y_mm;
  const char *plan_state = "FRONTIER";

  if (!SlamNav_GetPoseAndCell(&pose, &start_x, &start_y))
  {
    s_state = SLAM_NAV_STATE_NO_PATH;
    s_state_enter_tick_ms = HAL_GetTick();
    s_active = false;
    MotorControl_Stop();
    SlamNav_SendStatus("NO_PATH", "BAD_POSE");
    return;
  }

  MappingGrid_MarkRobotFree(&pose, SLAM_NAV_ROBOT_FREE_RADIUS);
  if (!MappingGrid_CopySnapshot(&s_snapshot))
  {
    s_state = SLAM_NAV_STATE_NO_PATH;
    s_state_enter_tick_ms = HAL_GetTick();
    s_active = false;
    MotorControl_Stop();
    SlamNav_SendStatus("NO_PATH", "SNAPSHOT");
    return;
  }

  taskENTER_CRITICAL();
  mode = s_mode;
  return_goal_x_mm = s_return_goal_x_mm;
  return_goal_y_mm = s_return_goal_y_mm;
  taskEXIT_CRITICAL();

  if (mode == SLAM_NAV_MODE_RETURN)
  {
    if (!MappingGrid_WorldToCell(return_goal_x_mm, return_goal_y_mm, &goal_x, &goal_y))
    {
      s_state = SLAM_NAV_STATE_NO_PATH;
      s_state_enter_tick_ms = HAL_GetTick();
      s_active = false;
      MotorControl_Stop();
      SlamNav_SendStatus("NO_PATH", "RETURN_GOAL");
      return;
    }

    if ((start_x == goal_x) && (start_y == goal_y))
    {
      s_state = SLAM_NAV_STATE_DONE;
      s_state_enter_tick_ms = HAL_GetTick();
      s_active = false;
      MotorControl_Stop();
      s_path.length = 1U;
      s_path.target.x = goal_x;
      s_path.target.y = goal_y;
      s_current_target_cell = s_path.target;
      SlamNav_SendStatus("DONE", "RETURN_HOME");
      return;
    }

    status = AstarPlanner_PlanToGoal(&s_snapshot, start_x, start_y, goal_x, goal_y, &s_path);
    plan_state = "RETURN";
  }
  else
  {
    status = AstarPlanner_PlanToFrontier(&s_snapshot, start_x, start_y, &s_path);
    if ((status != ASTAR_PLANNER_STATUS_OK) &&
        (status != ASTAR_PLANNER_STATUS_PATH_TRUNCATED))
    {
      if (!MappingGrid_WorldToCell(SLAM_NAV_CENTER_GOAL_X_MM, SLAM_NAV_CENTER_GOAL_Y_MM, &goal_x, &goal_y))
      {
        s_state = SLAM_NAV_STATE_NO_PATH;
        s_state_enter_tick_ms = HAL_GetTick();
        s_active = false;
        MotorControl_Stop();
        SlamNav_SendStatus("NO_PATH", "CENTER_GOAL");
        return;
      }

      if ((start_x == goal_x) && (start_y == goal_y))
      {
        s_state = SLAM_NAV_STATE_DONE;
        s_state_enter_tick_ms = HAL_GetTick();
        s_active = false;
        MotorControl_Stop();
        s_path.length = 1U;
        s_path.target.x = goal_x;
        s_path.target.y = goal_y;
        s_current_target_cell = s_path.target;
        SlamNav_SendStatus("DONE", "CENTER");
        return;
      }

      status = AstarPlanner_PlanToGoal(&s_snapshot, start_x, start_y, goal_x, goal_y, &s_path);
      plan_state = "CENTER";
    }
  }

  s_plan_seq++;
  if (((status != ASTAR_PLANNER_STATUS_OK) &&
       (status != ASTAR_PLANNER_STATUS_PATH_TRUNCATED)) ||
      (s_path.length < 2U))
  {
    s_state = SLAM_NAV_STATE_NO_PATH;
    s_state_enter_tick_ms = HAL_GetTick();
    s_active = false;
    MotorControl_Stop();
    SlamNav_SendStatus("NO_PATH", AstarPlanner_StatusName(status));
    return;
  }

  if (!SlamNav_SetTargetFromPath(SlamNav_SelectNextPathIndex(0U)))
  {
    s_state = SLAM_NAV_STATE_NO_PATH;
    s_state_enter_tick_ms = HAL_GetTick();
    s_active = false;
    MotorControl_Stop();
    SlamNav_SendStatus("NO_PATH", "TARGET");
    return;
  }

  s_state = SLAM_NAV_STATE_TURN;
  s_state_enter_tick_ms = HAL_GetTick();
  s_turn_settle_until_ms = 0U;
  s_path_revision = s_snapshot.revision;
  s_last_path_validate_tick_ms = HAL_GetTick();
  SlamNav_SendPath();
  SlamNav_SendStatus(plan_state, AstarPlanner_StatusName(status));
}

static void SlamNav_UpdateTurn(void)
{
  MappingGridPose_t pose;
  int32_t error_cdeg;
  uint16_t turn_pwm;
  uint32_t now = HAL_GetTick();

  if (!MappingGrid_GetPose(&pose))
  {
    s_state = SLAM_NAV_STATE_REPLAN;
    s_state_enter_tick_ms = HAL_GetTick();
    return;
  }

  error_cdeg = SlamNav_SignedHeadingErrorCdeg(s_target_heading_cdeg, pose.heading_cdeg);
  if (SlamNav_Abs32(error_cdeg) <= SLAM_NAV_TURN_TOL_CDEG)
  {
    MotorControl_Stop();
    if (s_turn_settle_until_ms == 0U)
    {
      s_turn_settle_until_ms = now + SLAM_NAV_TURN_SETTLE_MS;
      return;
    }

    if ((int32_t)(now - s_turn_settle_until_ms) < 0L)
    {
      return;
    }

    s_turn_settle_until_ms = 0U;
    if (s_escape_replan_after_turn)
    {
      s_escape_replan_after_turn = false;
      s_state = SLAM_NAV_STATE_REPLAN;
      s_state_enter_tick_ms = now;
      SlamNav_SendStatus("REPLAN", "AFTER_ESCAPE");
      return;
    }
    s_state = SLAM_NAV_STATE_DRIVE;
    s_state_enter_tick_ms = now;
    return;
  }

  s_turn_settle_until_ms = 0U;
  if ((now - s_state_enter_tick_ms) >= SLAM_NAV_TURN_TIMEOUT_MS)
  {
    MotorControl_Stop();
    s_state = SLAM_NAV_STATE_NO_PATH;
    s_state_enter_tick_ms = now;
    s_active = false;
    SlamNav_SendStatus("NO_PATH", "HEADING_TIMEOUT");
    return;
  }

  turn_pwm = SlamNav_ActivePwm(s_turn_pwm_permille, SLAM_NAV_DEFAULT_TURN_PWM, SLAM_NAV_MIN_TURN_PWM);
  if (error_cdeg > 0L)
  {
    MotorControl_SetTurnLeft(turn_pwm);
  }
  else
  {
    MotorControl_SetTurnRight(turn_pwm);
  }
}

static void SlamNav_UpdateDrive(void)
{
  MappingGridPose_t pose;
  uint32_t now = HAL_GetTick();
  uint8_t current_x;
  uint8_t current_y;
  int32_t dx_mm;
  int32_t dy_mm;
  int32_t distance_sq;
  int32_t error_cdeg;
  int32_t steering_permille;

  if (!SlamNav_GetPoseAndCell(&pose, &current_x, &current_y))
  {
    MotorControl_Stop();
    s_state = SLAM_NAV_STATE_REPLAN;
    s_state_enter_tick_ms = HAL_GetTick();
    return;
  }

  if (MappingGrid_GetCell(s_current_target_cell.x, s_current_target_cell.y) == MAPPING_GRID_CELL_OCCUPIED)
  {
    MotorControl_Stop();
    s_state = SLAM_NAV_STATE_REPLAN;
    s_state_enter_tick_ms = now;
    SlamNav_SendStatus("REPLAN", "TARGET_BLOCKED");
    return;
  }

  if (!SlamNav_PathStillTraversable(now))
  {
    MotorControl_Stop();
    s_state = SLAM_NAV_STATE_REPLAN;
    s_state_enter_tick_ms = now;
    SlamNav_SendStatus("REPLAN", "PATH_BLOCKED");
    return;
  }

  /* Front-block escape: lidar detects obstacle within safe distance while driving.
     Stop, choose turn direction by sector openness (mirrors command-96 avoidance),
     then replan after the turn completes. */
  {
    uint16_t front_mm;
    uint16_t safe_mm;

    taskENTER_CRITICAL();
    front_mm = s_front_min_distance_mm;
    safe_mm  = s_safe_distance_mm;
    taskEXIT_CRITICAL();

    if ((front_mm > 0U) && (front_mm < safe_mm))
    {
      SlamNavSectorStats_t sectors = SlamNav_GetSectorSnapshot(now);
      int32_t escape_heading;

      /* UINT16_MAX means the sector had no reading → treat as fully open (> safe_mm). */
      bool right_open = (sectors.right_mm > safe_mm);
      bool left_open  = (sectors.left_mm  > safe_mm);

      if (right_open && (!left_open || (sectors.right_mm >= sectors.left_mm)))
      {
        escape_heading = SlamNav_NormalizeHeadingCdeg(pose.heading_cdeg - 9000L);
      }
      else if (left_open)
      {
        escape_heading = SlamNav_NormalizeHeadingCdeg(pose.heading_cdeg + 9000L);
      }
      else
      {
        escape_heading = SlamNav_NormalizeHeadingCdeg(pose.heading_cdeg + 18000L);
      }

      MotorControl_Stop();
      s_target_heading_cdeg      = escape_heading;
      s_escape_replan_after_turn = true;
      s_state                    = SLAM_NAV_STATE_TURN;
      s_state_enter_tick_ms      = now;
      s_turn_settle_until_ms     = 0U;
      SlamNav_SendStatus("ESCAPE", "FRONT_BLOCK");
      return;
    }
  }

  dx_mm = s_target_x_mm - pose.x_mm;
  dy_mm = s_target_y_mm - pose.y_mm;
  distance_sq = (dx_mm * dx_mm) + (dy_mm * dy_mm);
  if (((current_x == s_current_target_cell.x) && (current_y == s_current_target_cell.y)) ||
      (distance_sq <= (SLAM_NAV_TARGET_RADIUS_MM * SLAM_NAV_TARGET_RADIUS_MM)))
  {
    MotorControl_Stop();
    s_state_enter_tick_ms = now;
    if ((s_path_index + 1U) < s_path.length)
    {
      if (SlamNav_SetTargetFromPath(SlamNav_SelectNextPathIndex(s_path_index)))
      {
        s_state = SLAM_NAV_STATE_TURN;
        s_turn_settle_until_ms = 0U;
        SlamNav_SendStatus("CELL", "ADVANCE");
        return;
      }
    }

    s_state = SLAM_NAV_STATE_PLAN;
    SlamNav_SendStatus("CELL", "REACHED");
    return;
  }

  error_cdeg = SlamNav_SignedHeadingErrorCdeg(s_target_heading_cdeg, pose.heading_cdeg);
  steering_permille = SlamNav_HeadingHoldSteer(error_cdeg);
  if (SlamNav_Abs32(error_cdeg) > SLAM_NAV_DRIVE_HEADING_TOL_CDEG)
  {
    MotorControl_Stop();
    s_state = SLAM_NAV_STATE_TURN;
    s_state_enter_tick_ms = now;
    s_turn_settle_until_ms = 0U;
    return;
  }

  MotorControl_SetForwardSteer(
      SlamNav_ActivePwm(s_drive_pwm_permille, SLAM_NAV_DEFAULT_DRIVE_PWM, SLAM_NAV_MIN_DRIVE_PWM),
      steering_permille);
}

static bool SlamNav_GetPoseAndCell(MappingGridPose_t *out_pose, uint8_t *out_x, uint8_t *out_y)
{
  MappingGridPose_t pose;

  if ((out_pose == NULL) || (out_x == NULL) || (out_y == NULL))
  {
    return false;
  }

  if (!MappingGrid_GetPose(&pose))
  {
    return false;
  }

  if (!MappingGrid_WorldToCell(pose.x_mm, pose.y_mm, out_x, out_y))
  {
    return false;
  }

  *out_pose = pose;
  return true;
}

static bool SlamNav_SetTargetFromPath(uint16_t path_index)
{
  AstarPlannerCell_t from;
  AstarPlannerCell_t to;

  if ((path_index == 0U) || (path_index >= s_path.length))
  {
    return false;
  }

  from = s_path.cells[path_index - 1U];
  to = s_path.cells[path_index];
  if (!MappingGrid_CellToWorld(to.x, to.y, &s_target_x_mm, &s_target_y_mm))
  {
    return false;
  }

  s_path_index = path_index;
  s_current_target_cell = to;
  s_target_heading_cdeg = SlamNav_CellHeadingCdeg(&from, &to);
  return true;
}

static uint16_t SlamNav_SelectNextPathIndex(uint16_t from_path_index)
{
  uint16_t target_index;
  int32_t heading_cdeg;

  if ((from_path_index + 1U) >= s_path.length)
  {
    return from_path_index;
  }

  target_index = from_path_index + 1U;
  heading_cdeg = SlamNav_CellHeadingCdeg(&s_path.cells[from_path_index], &s_path.cells[target_index]);

  while (((target_index + 1U) < s_path.length) &&
         ((target_index - from_path_index) < SLAM_NAV_MOTION_STEP_CELLS) &&
         (SlamNav_CellHeadingCdeg(&s_path.cells[target_index], &s_path.cells[target_index + 1U]) == heading_cdeg))
  {
    target_index++;
  }

  return target_index;
}

static bool SlamNav_PathStillTraversable(uint32_t now)
{
  uint32_t revision = MappingGrid_GetRevision();

  if (revision == s_path_revision)
  {
    return true;
  }

  if ((now - s_last_path_validate_tick_ms) < SLAM_NAV_PATH_VALIDATE_INTERVAL_MS)
  {
    return true;
  }
  s_last_path_validate_tick_ms = now;

  if (!MappingGrid_CopySnapshot(&s_snapshot))
  {
    return false;
  }

  if (!AstarPlanner_IsPathTraversable(&s_snapshot, &s_path, s_path_index))
  {
    return false;
  }

  s_path_revision = s_snapshot.revision;
  return true;
}

static bool SlamNav_IsFrontAngle(uint16_t angle_cdeg)
{
  return ((angle_cdeg <= SLAM_NAV_FRONT_SECTOR_CDEG) ||
          (angle_cdeg >= (uint16_t)(36000U - SLAM_NAV_FRONT_SECTOR_CDEG)));
}

static bool SlamNav_IsAngleNear(uint16_t angle_cdeg, uint16_t center_cdeg, uint16_t half_width_cdeg)
{
  int32_t error = SlamNav_SignedHeadingErrorCdeg((int32_t)center_cdeg, (int32_t)angle_cdeg);
  return SlamNav_Abs32(error) <= (int32_t)half_width_cdeg;
}

static int32_t SlamNav_NormalizeHeadingCdeg(int32_t heading_cdeg)
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

static int32_t SlamNav_SignedHeadingErrorCdeg(int32_t target_cdeg, int32_t current_cdeg)
{
  int32_t error = SlamNav_NormalizeHeadingCdeg(target_cdeg) -
      SlamNav_NormalizeHeadingCdeg(current_cdeg);

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

static int32_t SlamNav_Abs32(int32_t value)
{
  return (value < 0L) ? -value : value;
}

static uint16_t SlamNav_ClampPwm(uint16_t value)
{
  return (value > SLAM_NAV_MAX_PWM) ? SLAM_NAV_MAX_PWM : value;
}

static void SlamNav_ResetSectorStats(SlamNavSectorStats_t *stats)
{
  if (stats == NULL)
  {
    return;
  }

  stats->front_mm = UINT16_MAX;
  stats->right_mm = UINT16_MAX;
  stats->left_mm = UINT16_MAX;
  stats->back_mm = UINT16_MAX;
  stats->front_right_mm = UINT16_MAX;
  stats->front_left_mm = UINT16_MAX;
  stats->last_tick_ms = 0U;
}

static void SlamNav_UpdateSectorMin(uint16_t *value, uint16_t distance_mm)
{
  if (value == NULL)
  {
    return;
  }

  if ((*value == UINT16_MAX) || (distance_mm < *value))
  {
    *value = distance_mm;
  }
}

static void SlamNav_UpdateSectorStats(uint16_t robot_angle_cdeg, uint16_t distance_mm)
{
  uint32_t now = HAL_GetTick();

  taskENTER_CRITICAL();
  if (SlamNav_IsFrontAngle(robot_angle_cdeg))
  {
    SlamNav_UpdateSectorMin(&s_sector_current.front_mm, distance_mm);
  }
  if (SlamNav_IsAngleNear(robot_angle_cdeg, SLAM_NAV_RIGHT_CENTER_CDEG, SLAM_NAV_SIDE_SECTOR_CDEG))
  {
    SlamNav_UpdateSectorMin(&s_sector_current.right_mm, distance_mm);
  }
  if (SlamNav_IsAngleNear(robot_angle_cdeg, SLAM_NAV_LEFT_CENTER_CDEG, SLAM_NAV_SIDE_SECTOR_CDEG))
  {
    SlamNav_UpdateSectorMin(&s_sector_current.left_mm, distance_mm);
  }
  if (SlamNav_IsAngleNear(robot_angle_cdeg, SLAM_NAV_BACK_CENTER_CDEG, SLAM_NAV_SIDE_SECTOR_CDEG))
  {
    SlamNav_UpdateSectorMin(&s_sector_current.back_mm, distance_mm);
  }
  if (SlamNav_IsAngleNear(robot_angle_cdeg, SLAM_NAV_FRONT_RIGHT_CENTER_CDEG, SLAM_NAV_DIAGONAL_SECTOR_CDEG))
  {
    SlamNav_UpdateSectorMin(&s_sector_current.front_right_mm, distance_mm);
  }
  if (SlamNav_IsAngleNear(robot_angle_cdeg, SLAM_NAV_FRONT_LEFT_CENTER_CDEG, SLAM_NAV_DIAGONAL_SECTOR_CDEG))
  {
    SlamNav_UpdateSectorMin(&s_sector_current.front_left_mm, distance_mm);
  }
  s_sector_current.last_tick_ms = now;
  taskEXIT_CRITICAL();
}

static SlamNavSectorStats_t SlamNav_GetSectorSnapshot(uint32_t now)
{
  SlamNavSectorStats_t current;
  SlamNavSectorStats_t last;

  taskENTER_CRITICAL();
  current = s_sector_current;
  last = s_sector_last;
  taskEXIT_CRITICAL();

  if ((last.last_tick_ms != 0U) &&
      ((now - last.last_tick_ms) <= SLAM_NAV_SECTOR_STALE_MS))
  {
    return last;
  }

  return current;
}

static uint16_t SlamNav_ApplyForwardBodyOffset(uint16_t robot_angle_cdeg, uint16_t distance_mm)
{
  uint16_t offset_mm = 0U;

  if (distance_mm == 0U)
  {
    return 0U;
  }

  if (SlamNav_IsFrontAngle(robot_angle_cdeg))
  {
    offset_mm = SLAM_NAV_FRONT_BODY_OFFSET_MM;
  }
  else if (SlamNav_IsAngleNear(robot_angle_cdeg, SLAM_NAV_FRONT_RIGHT_CENTER_CDEG, SLAM_NAV_DIAGONAL_SECTOR_CDEG) ||
           SlamNav_IsAngleNear(robot_angle_cdeg, SLAM_NAV_FRONT_LEFT_CENTER_CDEG, SLAM_NAV_DIAGONAL_SECTOR_CDEG))
  {
    offset_mm = SLAM_NAV_FRONT_DIAGONAL_BODY_OFFSET_MM;
  }

  if (offset_mm == 0U)
  {
    return distance_mm;
  }

  return (distance_mm > offset_mm) ? (uint16_t)(distance_mm - offset_mm) : 0U;
}

static uint16_t SlamNav_ActivePwm(uint16_t configured_pwm, uint16_t fallback_pwm, uint16_t minimum_pwm)
{
  uint16_t pwm = (configured_pwm > 0U) ? configured_pwm : fallback_pwm;

  if ((pwm > 0U) && (pwm < minimum_pwm))
  {
    pwm = minimum_pwm;
  }

  return pwm;
}

static uint16_t SlamNav_ClampConfiguredPwm(uint16_t value, uint16_t maximum)
{
  value = SlamNav_ClampPwm(value);
  return (value > maximum) ? maximum : value;
}

static int32_t SlamNav_HeadingHoldSteer(int32_t error_cdeg)
{
  int32_t steer;

  if (SlamNav_Abs32(error_cdeg) <= SLAM_NAV_HEADING_HOLD_DEADBAND_CDEG)
  {
    return 0L;
  }

  steer = error_cdeg / SLAM_NAV_HEADING_HOLD_CDEG_PER_PWM;
  if (steer > SLAM_NAV_HEADING_HOLD_MAX_STEER)
  {
    return SLAM_NAV_HEADING_HOLD_MAX_STEER;
  }

  if (steer < -SLAM_NAV_HEADING_HOLD_MAX_STEER)
  {
    return -SLAM_NAV_HEADING_HOLD_MAX_STEER;
  }

  return steer;
}

static int32_t SlamNav_CellHeadingCdeg(const AstarPlannerCell_t *from, const AstarPlannerCell_t *to)
{
  int16_t dx;
  int16_t dy;

  if ((from == NULL) || (to == NULL))
  {
    return 0L;
  }

  dx = (int16_t)to->x - (int16_t)from->x;
  dy = (int16_t)to->y - (int16_t)from->y;

  if ((dx > 0) && (dy < 0))
  {
    return 4500L;
  }

  if ((dx < 0) && (dy < 0))
  {
    return 13500L;
  }

  if ((dx < 0) && (dy > 0))
  {
    return 22500L;
  }

  if ((dx > 0) && (dy > 0))
  {
    return 31500L;
  }

  if (dx > 0)
  {
    return 0L;
  }

  if (dx < 0)
  {
    return 18000L;
  }

  if (dy < 0)
  {
    return 9000L;
  }

  return 27000L;
}

static void SlamNav_SendStatus(const char *state, const char *reason)
{
  char line[128];
  uint32_t now = HAL_GetTick();
  bool low_priority_status = false;

  if (state == NULL)
  {
    state = SlamNav_StateName(s_state);
  }

  if (reason == NULL)
  {
    reason = "NONE";
  }

  low_priority_status =
      (strcmp(state, "CELL") == 0) ||
      (strcmp(state, "PLAN") == 0) ||
      (strcmp(state, "RETURN") == 0) ||
      (strcmp(state, "REPLAN") == 0);

  if (low_priority_status &&
      ((now - s_last_status_tick_ms) < SLAM_NAV_STATUS_INTERVAL_MS))
  {
    return;
  }
  s_last_status_tick_ms = now;

  (void)snprintf(
      line,
      sizeof(line),
      "SLAM state=%s reason=%s seq=%lu target=%u,%u path=%u front=%u\r\n",
      state,
      reason,
      (unsigned long)s_plan_seq,
      (unsigned int)s_current_target_cell.x,
      (unsigned int)s_current_target_cell.y,
      (unsigned int)s_path.length,
      (unsigned int)s_front_min_distance_mm);
  (void)BluetoothControl_SendText(line);
}

static void SlamNav_SendHeartbeatIfDue(void)
{
  char line[256];
  uint32_t now = HAL_GetTick();
  uint32_t revision = MappingGrid_GetRevision();
  uint16_t configured_safe_mm;
  uint16_t front_sector_mm;
  SlamNavSectorStats_t sectors;
  MappingGridPose_t pose;
  uint8_t cell_x = 0U;
  uint8_t cell_y = 0U;
  bool pose_ok;

  if ((now - s_last_heartbeat_tick_ms) < SLAM_NAV_HEARTBEAT_INTERVAL_MS)
  {
    return;
  }
  s_last_heartbeat_tick_ms = now;

  taskENTER_CRITICAL();
  configured_safe_mm = s_safe_distance_mm;
  taskEXIT_CRITICAL();
  sectors = SlamNav_GetSectorSnapshot(now);
  front_sector_mm = (sectors.front_mm == UINT16_MAX) ? 0U : sectors.front_mm;

  pose_ok = SlamNav_GetPoseAndCell(&pose, &cell_x, &cell_y);
  (void)snprintf(
      line,
      sizeof(line),
      "SLAM HB state=%s seq=%lu cell=%u,%u target=%u,%u path_i=%u path_len=%u front=%u rev=%lu path_rev=%lu safe=%u sec=%u,%u,%u,%u pose=%ld,%ld,%ld ok=%u\r\n",
      SlamNav_StateName(s_state),
      (unsigned long)s_plan_seq,
      (unsigned int)cell_x,
      (unsigned int)cell_y,
      (unsigned int)s_current_target_cell.x,
      (unsigned int)s_current_target_cell.y,
      (unsigned int)s_path_index,
      (unsigned int)s_path.length,
      (unsigned int)front_sector_mm,
      (unsigned long)revision,
      (unsigned long)s_path_revision,
      (unsigned int)configured_safe_mm,
      (unsigned int)sectors.front_mm,
      (unsigned int)sectors.right_mm,
      (unsigned int)sectors.left_mm,
      (unsigned int)sectors.back_mm,
      pose_ok ? (long)pose.x_mm : 0L,
      pose_ok ? (long)pose.y_mm : 0L,
      pose_ok ? (long)pose.heading_cdeg : 0L,
      (unsigned int)pose_ok);
  (void)BluetoothControl_SendText(line);
}

static void SlamNav_SendPath(void)
{
  char line[128];
  uint16_t index = 0U;
  uint32_t now = HAL_GetTick();
  uint32_t revision = MappingGrid_GetRevision();
  bool same_path_target;

  same_path_target =
      s_last_path_tx_valid &&
      (s_last_path_tx_target.x == s_path.target.x) &&
      (s_last_path_tx_target.y == s_path.target.y) &&
      (s_last_path_tx_length == s_path.length);

  if (same_path_target &&
      ((now - s_last_path_tx_tick_ms) < SLAM_NAV_PATH_TX_INTERVAL_MS))
  {
    return;
  }

  s_last_path_tx_tick_ms = now;
  s_last_path_tx_target = s_path.target;
  s_last_path_tx_length = s_path.length;
  s_last_path_tx_valid = true;

  (void)snprintf(
      line,
      sizeof(line),
      "PATH BEGIN seq=%lu rev=%lu len=%u target=%u,%u\r\n",
      (unsigned long)s_plan_seq,
      (unsigned long)revision,
      (unsigned int)s_path.length,
      (unsigned int)s_path.target.x,
      (unsigned int)s_path.target.y);
  (void)BluetoothControl_SendText(line);

  while (index < s_path.length)
  {
    uint16_t chunk_start = index;
    uint16_t used;

    (void)snprintf(
        line,
        sizeof(line),
        "PATH CHUNK seq=%lu idx=%u data=",
        (unsigned long)s_plan_seq,
        (unsigned int)chunk_start);
    used = (uint16_t)strlen(line);

    while ((index < s_path.length) &&
           ((index - chunk_start) < SLAM_NAV_PATH_CHUNK_CELLS) &&
           (used < (sizeof(line) - 10U)))
    {
      int written = snprintf(
          &line[used],
          sizeof(line) - used,
          "%u,%u;",
          (unsigned int)s_path.cells[index].x,
          (unsigned int)s_path.cells[index].y);

      if (written <= 0)
      {
        break;
      }

      used = (uint16_t)(used + (uint16_t)written);
      index++;
    }

    if (used < (sizeof(line) - 2U))
    {
      line[used++] = '\r';
      line[used++] = '\n';
      line[used] = '\0';
    }
    else
    {
      line[sizeof(line) - 3U] = '\r';
      line[sizeof(line) - 2U] = '\n';
      line[sizeof(line) - 1U] = '\0';
    }

    (void)BluetoothControl_SendText(line);
  }

  (void)snprintf(
      line,
      sizeof(line),
      "PATH END seq=%lu\r\n",
      (unsigned long)s_plan_seq);
  (void)BluetoothControl_SendText(line);
}
