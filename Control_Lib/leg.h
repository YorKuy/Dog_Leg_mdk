#ifndef __CONTROL_LEG_H__
#define __CONTROL_LEG_H__

#include <stdint.h>
#include "DM.h"

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

#endif
