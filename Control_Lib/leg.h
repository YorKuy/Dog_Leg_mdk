#ifndef __CONTROL_LEG_H__
#define __CONTROL_LEG_H__

#include <stdint.h>
#include "DM.h"

typedef enum
{
  LEG_DYNAMIC_NORMAL = 0,
  LEG_DYNAMIC_SLOPE_CANDIDATE,
  LEG_DYNAMIC_SUPPORT_HOLD,
  LEG_DYNAMIC_UNLOAD_CATCH,
  LEG_DYNAMIC_SETTLE
} LegDynamicState;

// Ozone can change these switches and gains without rebuilding.
extern volatile uint8_t leg_enable_accel_feedforward;
extern volatile uint8_t leg_enable_forward_jerk_limit;
extern volatile uint8_t leg_enable_height_damping;
extern volatile uint8_t leg_enable_slope_hold;
extern volatile uint8_t leg_enable_unload_catch;
extern volatile uint8_t leg_enable_roll_balance;
extern volatile float leg_accel_ff_accel_gain;
extern volatile float leg_accel_ff_brake_gain;
extern volatile float leg_accel_ff_limit;
extern volatile float leg_height_damping_gain;
extern volatile float leg_height_hold_kp;
extern volatile float leg_height_hold_kd;
extern volatile float leg_height_hold_limit;
extern volatile float leg_roll_balance_kp;
extern volatile float leg_roll_balance_kd;
extern volatile float leg_roll_balance_limit;
extern volatile float leg_roll_balance_direction;
extern volatile uint32_t leg_dm_pair_send_ok_count;
extern volatile uint32_t leg_dm_pair_send_fail_count;
extern volatile uint16_t leg_dm_pair_send_consecutive_fail;

typedef struct
{
  uint8_t yk_mode;
  uint8_t xtl_flag;
  int16_t ch3;
  uint8_t key_e;
  uint8_t key_ctrl;

  float gimbal_roll;
  float gimbal_pitch;
  float gimbal_roll_acc;
  float gimbal_pitch_acc;

  float fb_real_speed;
  float lr_real_speed;
  float chassis_ch0_real;
  float chassis_ch1_real;

  float sin_theta;
  float cos_theta;

  float dt;
  float forward_cmd;
  float forward_accel;
  float wheel_stall_ratio;
  float pitch_data_age;
  float roll_data_age;

  MOTOR_DM *left_motor;
  MOTOR_DM *right_motor;
} LegControlInput;

typedef struct
{
  float left_torque;
  float right_torque;
  float mit_kd;
  float roll_cmd;
  float pitch_cmd;
  float smc_cmd;
  float accel_ff_cmd;
  float height_damping_cmd;
  float height_hold_cmd;
  float roll_balance_cmd;
  float height;
  float height_speed;
  uint8_t dynamic_state;
} LegControlOutput;

void Leg_Control_Config_MotorLimit(MOTOR_DM *left_motor, MOTOR_DM *right_motor);
uint8_t Leg_Control_Mode_Update(uint8_t player_mode_active, uint8_t x_key_pressed);
uint8_t Leg_Control_Mode_Is_Enabled(void);
void Leg_Control_Update(const LegControlInput *input);
void Leg_SMC_Control(const LegControlInput *input);
void Leg_Control_Get_Output(LegControlOutput *output);
float Leg_Control_Get_Left_Torque(void);
float Leg_Control_Get_Right_Torque(void);
float Leg_Control_Get_Mit_Kd(void);
float Leg_Control_Get_Roll_Cmd(void);
void Leg_Control_Report_OutputPairResult(uint8_t success);

#endif
