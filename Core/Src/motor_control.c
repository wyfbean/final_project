#include "motor_control.h"

#include "FreeRTOS.h"
#include "task.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

#define MOTOR_PWM_TIMER_MAX      999U
#define MOTOR_PWM_CH1_MASK       0x01U
#define MOTOR_PWM_CH2_MASK       0x02U
#define MOTOR_PWM_CH3_MASK       0x04U
#define MOTOR_PWM_CH4_MASK       0x08U
#define MOTOR_ENCODER_REPORT_MS  100U
#define MOTOR_MODE_STOP          0U
#define MOTOR_MODE_FORWARD       1U
#define MOTOR_MODE_TURN_LEFT     2U
#define MOTOR_MODE_TURN_RIGHT    3U
#define MOTOR_MODE_BRAKE         4U
#define MOTOR_BRAKE_DUTY_PERMILLE 1000U
#define MOTOR_BRAKE_HOLD_MS      160U
#define MOTOR_STRAIGHT_KP        2L
#define MOTOR_STRAIGHT_KI        1L
#define MOTOR_STRAIGHT_KD        2L
#define MOTOR_STRAIGHT_GAIN_DIV  4L
#define MOTOR_STRAIGHT_MAX_CORR  90L
#define MOTOR_STRAIGHT_MAX_I     80L
#define MOTOR_STRAIGHT_DEADBAND  1L
#define MOTOR_RIGHT_FWD_BIAS     15L
#define MOTOR_FORWARD_MAX_STEER  90L
#define MOTOR_STRAIGHT_MIN_DUTY  0U
#define MOTOR_STRAIGHT_MIN_COUNTS 4L

static MotorControlState_t s_motor_state;
static uint16_t s_last_left_counter;
static uint16_t s_last_right_counter;
static int32_t s_left_window_delta;
static int32_t s_right_window_delta;
static int32_t s_straight_integral;
static int32_t s_straight_prev_error;
static int32_t s_balance_correction_permille;
static int32_t s_forward_steer_permille;
static uint32_t s_last_encoder_report_tick_ms;
static uint32_t s_brake_release_tick_ms;

static bool MotorControl_StartChannel(uint32_t channel);
static void MotorControl_UpdateEncoderDebug(void);
static void MotorControl_UpdateBrakeRelease(void);
static uint8_t MotorControl_ReadLeftEncoderRaw(void);
static uint8_t MotorControl_ReadRightEncoderRaw(void);
static void MotorControl_ApplyForwardControl(void);
static void MotorControl_ApplyRaw(uint16_t ch1_right_reverse,
                                  uint16_t ch2_right_forward,
                                  uint16_t ch3_left_forward,
                                  uint16_t ch4_left_reverse);
static void MotorControl_ApplyForwardDuty(uint16_t left_duty_permille, uint16_t right_duty_permille);
static void MotorControl_ApplyBrake(uint16_t duty_permille);
static void MotorControl_Coast(void);
static void MotorControl_ResetEncoderWindow(uint32_t now);
static void MotorControl_UpdateStraightCorrection(void);
static int16_t MotorControl_ComputeDelta(uint16_t current, uint16_t previous);
static int32_t MotorControl_Abs32(int32_t value);
static int32_t MotorControl_Clamp32(int32_t value, int32_t min_value, int32_t max_value);
static uint16_t MotorControl_PermilleToCompare(uint16_t duty_permille);

bool MotorControl_Init(void)
{
  bool ok = true;

  if (s_motor_state.ready)
  {
    return true;
  }

  ok = MotorControl_StartChannel(TIM_CHANNEL_1) && ok;
  ok = MotorControl_StartChannel(TIM_CHANNEL_2) && ok;
  ok = MotorControl_StartChannel(TIM_CHANNEL_3) && ok;
  ok = MotorControl_StartChannel(TIM_CHANNEL_4) && ok;

  s_last_left_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFU);
  s_last_right_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) & 0xFFFFU);
  s_motor_state.left_counter = s_last_left_counter;
  s_motor_state.right_counter = s_last_right_counter;
  s_motor_state.left_encoder_raw = MotorControl_ReadLeftEncoderRaw();
  s_motor_state.right_encoder_raw = MotorControl_ReadRightEncoderRaw();

  MotorControl_Stop();
  s_motor_state.ready = ok;
  return ok;
}

void MotorControl_Stop(void)
{
  uint32_t now = HAL_GetTick();

  MotorControl_ApplyBrake(MOTOR_BRAKE_DUTY_PERMILLE);
  s_motor_state.forward_active = false;
  s_motor_state.mode = MOTOR_MODE_BRAKE;
  s_motor_state.duty_permille = 0U;
  s_motor_state.left_duty_permille = 0U;
  s_motor_state.right_duty_permille = 0U;
  s_motor_state.left_delta = 0;
  s_motor_state.right_delta = 0;
  s_motor_state.balance_error = 0;
  s_motor_state.correction_permille = 0;
  s_motor_state.active_pwm_mask = MOTOR_PWM_CH1_MASK | MOTOR_PWM_CH2_MASK | MOTOR_PWM_CH3_MASK | MOTOR_PWM_CH4_MASK;
  s_left_window_delta = 0;
  s_right_window_delta = 0;
  s_straight_integral = 0L;
  s_straight_prev_error = 0L;
  s_balance_correction_permille = 0L;
  s_forward_steer_permille = 0L;
  s_brake_release_tick_ms = now + MOTOR_BRAKE_HOLD_MS;
}

void MotorControl_SetForward(uint16_t duty_permille)
{
  MotorControl_SetForwardSteer(duty_permille, 0L);
}

void MotorControl_SetForwardSteer(uint16_t duty_permille, int32_t steering_permille)
{
  uint32_t now = HAL_GetTick();
  bool reset_encoder_pid = false;

  if (duty_permille > 1000U)
  {
    duty_permille = 1000U;
  }

  if ((duty_permille > 0U) && (duty_permille < MOTOR_STRAIGHT_MIN_DUTY))
  {
    duty_permille = MOTOR_STRAIGHT_MIN_DUTY;
  }

  steering_permille = MotorControl_Clamp32(
      steering_permille,
      -MOTOR_FORWARD_MAX_STEER,
      MOTOR_FORWARD_MAX_STEER);

  if ((duty_permille > 0U) &&
      (s_motor_state.mode == MOTOR_MODE_FORWARD) &&
      (s_motor_state.duty_permille == duty_permille) &&
      (s_forward_steer_permille == steering_permille))
  {
    return;
  }

  if ((s_motor_state.mode != MOTOR_MODE_FORWARD) ||
      (s_motor_state.duty_permille != duty_permille) ||
      (duty_permille == 0U))
  {
    reset_encoder_pid = true;
    s_straight_integral = 0L;
    s_straight_prev_error = 0L;
    s_balance_correction_permille = 0L;
  }

  s_forward_steer_permille = (duty_permille > 0U) ? steering_permille : 0L;
  s_motor_state.forward_active = (duty_permille > 0U);
  s_motor_state.mode = (duty_permille > 0U) ? MOTOR_MODE_FORWARD : MOTOR_MODE_STOP;
  s_motor_state.duty_permille = duty_permille;
  MotorControl_ApplyForwardControl();

  if (reset_encoder_pid)
  {
    MotorControl_ResetEncoderWindow(now);
  }
}

void MotorControl_SetTurnLeft(uint16_t duty_permille)
{
  if (duty_permille > 1000U)
  {
    duty_permille = 1000U;
  }

  MotorControl_ApplyRaw(0U,
                        MotorControl_PermilleToCompare(duty_permille),
                        0U,
                        MotorControl_PermilleToCompare(duty_permille));
  s_motor_state.forward_active = false;
  s_motor_state.mode = (duty_permille > 0U) ? MOTOR_MODE_TURN_LEFT : MOTOR_MODE_STOP;
  s_motor_state.duty_permille = duty_permille;
  s_motor_state.left_duty_permille = duty_permille;
  s_motor_state.right_duty_permille = duty_permille;
  s_motor_state.balance_error = 0;
  s_motor_state.correction_permille = 0;
  s_motor_state.active_pwm_mask = (duty_permille > 0U) ? (MOTOR_PWM_CH2_MASK | MOTOR_PWM_CH4_MASK) : 0U;
  s_straight_integral = 0L;
  s_straight_prev_error = 0L;
  s_balance_correction_permille = 0L;
  s_forward_steer_permille = 0L;
  MotorControl_ResetEncoderWindow(HAL_GetTick());
}

void MotorControl_SetTurnRight(uint16_t duty_permille)
{
  if (duty_permille > 1000U)
  {
    duty_permille = 1000U;
  }

  MotorControl_ApplyRaw(MotorControl_PermilleToCompare(duty_permille),
                        0U,
                        MotorControl_PermilleToCompare(duty_permille),
                        0U);
  s_motor_state.forward_active = false;
  s_motor_state.mode = (duty_permille > 0U) ? MOTOR_MODE_TURN_RIGHT : MOTOR_MODE_STOP;
  s_motor_state.duty_permille = duty_permille;
  s_motor_state.left_duty_permille = duty_permille;
  s_motor_state.right_duty_permille = duty_permille;
  s_motor_state.balance_error = 0;
  s_motor_state.correction_permille = 0;
  s_motor_state.active_pwm_mask = (duty_permille > 0U) ? (MOTOR_PWM_CH1_MASK | MOTOR_PWM_CH3_MASK) : 0U;
  s_straight_integral = 0L;
  s_straight_prev_error = 0L;
  s_balance_correction_permille = 0L;
  s_forward_steer_permille = 0L;
  MotorControl_ResetEncoderWindow(HAL_GetTick());
}

void MotorControl_UpdateButtons(void)
{
  if (!s_motor_state.ready)
  {
    return;
  }

  s_motor_state.button_mask = 0U;
  MotorControl_UpdateBrakeRelease();
  MotorControl_UpdateEncoderDebug();
}

bool MotorControl_GetState(MotorControlState_t *out_state)
{
  if (out_state == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *out_state = s_motor_state;
  taskEXIT_CRITICAL();
  return s_motor_state.ready;
}

static bool MotorControl_StartChannel(uint32_t channel)
{
  if (HAL_TIM_PWM_Start(&htim3, channel) != HAL_OK)
  {
    s_motor_state.start_errors++;
    return false;
  }

  return true;
}

static void MotorControl_UpdateEncoderDebug(void)
{
  uint16_t left_now = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFU);
  uint16_t right_now = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) & 0xFFFFU);
  int16_t left_delta = MotorControl_ComputeDelta(left_now, s_last_left_counter);
  int16_t right_delta = MotorControl_ComputeDelta(right_now, s_last_right_counter);
  uint32_t now = HAL_GetTick();

  s_last_left_counter = left_now;
  s_last_right_counter = right_now;
  s_motor_state.left_counter = left_now;
  s_motor_state.right_counter = right_now;
  s_motor_state.left_encoder_raw = MotorControl_ReadLeftEncoderRaw();
  s_motor_state.right_encoder_raw = MotorControl_ReadRightEncoderRaw();
  s_left_window_delta += left_delta;
  s_right_window_delta += right_delta;
  if ((now - s_last_encoder_report_tick_ms) >= MOTOR_ENCODER_REPORT_MS)
  {
    s_motor_state.left_delta = (int16_t)s_left_window_delta;
    s_motor_state.right_delta = (int16_t)s_right_window_delta;
    if (s_motor_state.mode == MOTOR_MODE_FORWARD)
    {
      MotorControl_UpdateStraightCorrection();
    }
    else
    {
      s_motor_state.balance_error = s_left_window_delta - s_right_window_delta;
      s_motor_state.correction_permille = 0;
    }
    s_left_window_delta = 0;
    s_right_window_delta = 0;
    s_last_encoder_report_tick_ms = now;
  }
  s_motor_state.control_ticks++;
}

static void MotorControl_UpdateBrakeRelease(void)
{
  uint32_t now;

  if (s_motor_state.mode != MOTOR_MODE_BRAKE)
  {
    return;
  }

  now = HAL_GetTick();
  if ((int32_t)(now - s_brake_release_tick_ms) < 0L)
  {
    return;
  }

  MotorControl_Coast();
}

static uint8_t MotorControl_ReadLeftEncoderRaw(void)
{
  uint8_t raw = 0U;

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) { raw |= 0x01U; }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET) { raw |= 0x02U; }

  return raw;
}

static uint8_t MotorControl_ReadRightEncoderRaw(void)
{
  uint8_t raw = 0U;

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) { raw |= 0x01U; }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) { raw |= 0x02U; }

  return raw;
}

static void MotorControl_ApplyRaw(uint16_t ch1_right_reverse,
                                  uint16_t ch2_right_forward,
                                  uint16_t ch3_left_forward,
                                  uint16_t ch4_left_reverse)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ch1_right_reverse);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ch2_right_forward);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ch3_left_forward);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ch4_left_reverse);
}

static void MotorControl_ApplyForwardDuty(uint16_t left_duty_permille, uint16_t right_duty_permille)
{
  MotorControl_ApplyRaw(0U,
                        MotorControl_PermilleToCompare(right_duty_permille),
                        MotorControl_PermilleToCompare(left_duty_permille),
                        0U);
}

static void MotorControl_ApplyForwardControl(void)
{
  int32_t total_steer = s_balance_correction_permille + s_forward_steer_permille;
  int32_t left_duty = (int32_t)s_motor_state.duty_permille - total_steer;
  int32_t right_duty = (int32_t)s_motor_state.duty_permille + MOTOR_RIGHT_FWD_BIAS + total_steer;

  if ((s_motor_state.mode != MOTOR_MODE_FORWARD) ||
      (s_motor_state.duty_permille == 0U))
  {
    s_motor_state.left_duty_permille = 0U;
    s_motor_state.right_duty_permille = 0U;
    s_motor_state.correction_permille = 0L;
    s_motor_state.active_pwm_mask = 0U;
    MotorControl_ApplyForwardDuty(0U, 0U);
    return;
  }

  left_duty = MotorControl_Clamp32(left_duty, MOTOR_STRAIGHT_MIN_DUTY, 1000L);
  right_duty = MotorControl_Clamp32(right_duty, MOTOR_STRAIGHT_MIN_DUTY, 1000L);

  s_motor_state.left_duty_permille = (uint16_t)left_duty;
  s_motor_state.right_duty_permille = (uint16_t)right_duty;
  s_motor_state.correction_permille = MOTOR_RIGHT_FWD_BIAS + total_steer;
  s_motor_state.active_pwm_mask = MOTOR_PWM_CH2_MASK | MOTOR_PWM_CH3_MASK;

  MotorControl_ApplyForwardDuty(s_motor_state.left_duty_permille,
                                s_motor_state.right_duty_permille);
}

static void MotorControl_ApplyBrake(uint16_t duty_permille)
{
  uint16_t compare = MotorControl_PermilleToCompare(duty_permille);

  MotorControl_ApplyRaw(compare, compare, compare, compare);
}

static void MotorControl_Coast(void)
{
  MotorControl_ApplyRaw(0U, 0U, 0U, 0U);
  s_motor_state.mode = MOTOR_MODE_STOP;
  s_motor_state.active_pwm_mask = 0U;
  s_brake_release_tick_ms = 0U;
}

static void MotorControl_ResetEncoderWindow(uint32_t now)
{
  s_last_left_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFU);
  s_last_right_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) & 0xFFFFU);
  s_left_window_delta = 0;
  s_right_window_delta = 0;
  s_last_encoder_report_tick_ms = now;
}

static void MotorControl_UpdateStraightCorrection(void)
{
  int32_t left_travel = MotorControl_Abs32(s_left_window_delta);
  int32_t right_travel = MotorControl_Abs32(s_right_window_delta);
  int32_t error = left_travel - right_travel;
  int32_t derivative;
  int32_t pid_sum;
  int32_t total_travel = left_travel + right_travel;

  if (total_travel < MOTOR_STRAIGHT_MIN_COUNTS)
  {
    s_motor_state.balance_error = error;
    return;
  }

  if (MotorControl_Abs32(error) <= MOTOR_STRAIGHT_DEADBAND)
  {
    error = 0L;
    s_straight_integral = 0L;
  }
  else
  {
    s_straight_integral = MotorControl_Clamp32(s_straight_integral + error,
                                               -MOTOR_STRAIGHT_MAX_I,
                                               MOTOR_STRAIGHT_MAX_I);
  }

  derivative = error - s_straight_prev_error;
  s_straight_prev_error = error;
  pid_sum = (error * MOTOR_STRAIGHT_KP) +
      (s_straight_integral * MOTOR_STRAIGHT_KI) +
      (derivative * MOTOR_STRAIGHT_KD);
  s_balance_correction_permille = MotorControl_Clamp32(
      pid_sum / MOTOR_STRAIGHT_GAIN_DIV,
      -MOTOR_STRAIGHT_MAX_CORR,
      MOTOR_STRAIGHT_MAX_CORR);

  s_motor_state.balance_error = error;
  MotorControl_ApplyForwardControl();
}

static int16_t MotorControl_ComputeDelta(uint16_t current, uint16_t previous)
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

static int32_t MotorControl_Abs32(int32_t value)
{
  return (value < 0L) ? -value : value;
}

static int32_t MotorControl_Clamp32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

static uint16_t MotorControl_PermilleToCompare(uint16_t duty_permille)
{
  return (uint16_t)(((uint32_t)duty_permille * MOTOR_PWM_TIMER_MAX) / 1000U);
}
