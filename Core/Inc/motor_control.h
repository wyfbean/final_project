#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

typedef struct
{
  bool ready;
  bool forward_active;
  uint8_t mode;
  uint16_t duty_permille;
  uint16_t left_duty_permille;
  uint16_t right_duty_permille;
  uint16_t left_counter;
  uint16_t right_counter;
  uint8_t left_encoder_raw;
  uint8_t right_encoder_raw;
  int16_t left_delta;
  int16_t right_delta;
  int32_t balance_error;
  int32_t correction_permille;
  uint8_t button_mask;
  uint8_t active_pwm_mask;
  uint32_t start_errors;
  uint32_t control_ticks;
} MotorControlState_t;

bool MotorControl_Init(void);
void MotorControl_Stop(void);
void MotorControl_SetForward(uint16_t duty_permille);
void MotorControl_SetForwardSteer(uint16_t duty_permille, int32_t steering_permille);
void MotorControl_SetTurnLeft(uint16_t duty_permille);
void MotorControl_SetTurnRight(uint16_t duty_permille);
void MotorControl_UpdateButtons(void);
bool MotorControl_GetState(MotorControlState_t *out_state);

#endif /* MOTOR_CONTROL_H */
