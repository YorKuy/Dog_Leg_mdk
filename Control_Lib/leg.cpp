#include "leg.h"
#include "SMC.h"
#include "main.h"

#include <math.h>

typedef float f;
typedef uint8_t u8;

// 腿部机械角度范围
#define LEFT_LEG_MAX_MANG -0.59207F
#define LEFT_LEG_MIN_MANG -1.34095F
#define RIGHT_LEG_MAX_MANG -2.59529F
#define RIGHT_LEG_MIN_MANG -1.92694F

// 电机阻尼与输出保护
#define LEG_DM_MIT_KD 3.5f
#define LEG_JOINT_DAMPING_GAIN 0.60f
#define LEG_JOINT_DAMPING_SPEED_DEADZONE 0.20f

// 两轴平衡参数。软件 roll/pitch 对应的物理轴由 main.c 根据装配关系交换。
#define LEG_PITCH_TARGET_ANGLE 0.0f
#define LEG_PITCH_BALANCE_SIGN -1.0f
#define LEG_PITCH_OUTPUT_LIMIT 18.0f
#define LEG_ROLL_BALANCE_SIGN -1.0f
#define LEG_ROLL_KEEP_TARGET_ANGLE 0.0f
#define LEG_SMC_OUTPUT_SLEW_STEP 5.0f

// FAST/PLAYER 腿长串级 PID
#define LEG_FAST_HEIGHT_KP 242.0f
#define LEG_FAST_HEIGHT_KD 0.60f
#define LEG_FAST_HEIGHT_LIMIT 38.0f
#define LEG_FAST_HEIGHT_UP_FF 5.0f
#define LEG_FAST_HEIGHT_STEP_DEADZONE 0.00005f
#define LEG_FAST_HEIGHT_TARGET_STEP_DIV 6600.0f
#define LEG_FAST_HEIGHT_FF_ADD 0.10f
#define LEG_FAST_HEIGHT_FF_DECAY 0.96f
#define LEG_FAST_HEIGHT_FF_LIMIT 50.0f
#define LEG_FAST_HEIGHT_OUT_LIMIT 40.0f
#define LEG_FAST_HEIGHT_ERROR_DEADZONE 0.010f
#define LEG_FAST_HEIGHT_ERROR_GROW_DEADZONE 0.0003f
#define LEG_FAST_HEIGHT_PID_FULL_RATIO 0.90f

static PID_class left_control_mang(120.0f, 0.0f, 0.0f, 84.0f, 0.0f, 0.0f, 84.0f, 0.06f, 0.0f),
    left_control_sp(0.32f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f, 20.0f, 0.20f, 0.0f),
    right_control_mang(120.0f, 0.0f, 0.0f, 84.0f, 0.0f, 0.0f, 84.0f, 0.06f, 0.0f),
    right_control_sp(0.32f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f, 20.0f, 0.20f, 0.0f);

static SMC_PITCH leg_roll_smc(42, 55, 1.05f, 0.5f, 8000, 0.8f, 1.0f);
static UpDown_check_class leg_mode_key(0);
static u8 leg_control_mode = 0;
static f leg_dm_mit_kd = LEG_DM_MIT_KD;
static f leg_roll_cmd;
static f leg_pitch_cmd;
static f left_mang_ff;
static f right_mang_ff;
static f left_mang_last_error;
static f right_mang_last_error;
static LegControlOutput leg_output = {0.0f, 0.0f, LEG_DM_MIT_KD, 0.0f, 0.0f};

// 将 SMC 原始输出换算为电机力矩。
static f torque_return(f u)
{
  f current = u / (16384.0f / 3.0f);
  return current * 0.741f * 40.0f;
}

static f leg_min(f a, f b)
{
  return (a < b) ? a : b;
}

static f leg_max(f a, f b)
{
  return (a > b) ? a : b;
}

static void leg_sync_output(void)
{
  leg_output.mit_kd = leg_dm_mit_kd;
  leg_output.roll_cmd = leg_roll_cmd;
  leg_output.pitch_cmd = leg_pitch_cmd;
}
// 加入关节速度阻尼并限制最终力矩。
static f leg_apply_joint_damping_custom(f torque_cmd,
                                        MOTOR_DM *motor,
                                        f torque_limit,
                                        f damping_gain,
                                        f speed_deadzone)
{
  f motor_sp = (motor != 0) ? motor->sp : 0;
  if (fabsf(motor_sp) < speed_deadzone)
    motor_sp = 0;
  return LIMIT(torque_cmd - damping_gain * motor_sp, -torque_limit, torque_limit);
}

static f leg_remaining_output(f torque_limit, f used_output)
{
  f remaining = torque_limit - fabsf(used_output);
  return (remaining > 0.0f) ? remaining : 0.0f;
}
// 镜像安装混控：roll 左右相反，pitch 左右相同。
static void leg_set_balance_output_custom(const LegControlInput *input,
                                          f roll_cmd,
                                          f pitch_cmd,
                                          f torque_limit,
                                          f damping_gain,
                                          f speed_deadzone)
{
  f roll_out = LIMIT(LEG_ROLL_BALANCE_SIGN * roll_cmd,
                     -torque_limit,
                      torque_limit);

  f pitch_limit = leg_min(LEG_PITCH_OUTPUT_LIMIT, torque_limit);
  f pitch_out = LIMIT(LEG_PITCH_BALANCE_SIGN * pitch_cmd,
                      -pitch_limit,
                       pitch_limit);

  // 当前平衡优先保证 roll，pitch 使用剩余输出空间。
  f pitch_allow = leg_remaining_output(torque_limit, roll_out);
  pitch_out = LIMIT(pitch_out, -pitch_allow, pitch_allow);

  f left_leg_cmd = pitch_out + roll_out;
  f right_leg_cmd = pitch_out - roll_out;

  leg_output.left_torque =
      leg_apply_joint_damping_custom(left_leg_cmd,
                                     input->left_motor,
                                     torque_limit,
                                     damping_gain,
                                     speed_deadzone);

  leg_output.right_torque =
      leg_apply_joint_damping_custom(right_leg_cmd,
                                     input->right_motor,
                                     torque_limit,
                                     damping_gain,
                                     speed_deadzone);
}
// 使用默认关节阻尼生成平衡输出。
static void leg_set_balance_output(const LegControlInput *input,
                                   f roll_cmd,
                                   f pitch_cmd,
                                   f torque_limit)
{
  leg_set_balance_output_custom(input,
                                roll_cmd,
                                pitch_cmd,
                                torque_limit,
                                LEG_JOINT_DAMPING_GAIN,
                                LEG_JOINT_DAMPING_SPEED_DEADZONE);
}
// 腿长角度外环、速度内环及误差增长前馈。
static f leg_fast_mang_cmd(PID_class *mang_pid,
                           PID_class *sp_pid,
                           f target_mang,
                           f now_mang,
                           f now_sp,
                           f *ff_out,
                           f *last_error)
{
  f error = target_mang - now_mang;
  f abs_error = fabsf(error);
  f abs_last_error = fabsf(*last_error);

  mang_pid->PID_new_update(target_mang, now_mang);

  if (abs_error < LEG_FAST_HEIGHT_ERROR_DEADZONE)
  {
    *ff_out = 0;
  }
  else if (error * (*last_error) < 0)
  {
    *ff_out = 0;
  }
  else if (abs_error > abs_last_error + LEG_FAST_HEIGHT_ERROR_GROW_DEADZONE ||
           fabsf(mang_pid->OUT_PID) > LEG_FAST_HEIGHT_LIMIT * LEG_FAST_HEIGHT_PID_FULL_RATIO)
  {
    *ff_out += (error > 0) ? LEG_FAST_HEIGHT_FF_ADD : -LEG_FAST_HEIGHT_FF_ADD;
  }
  else
  {
    *ff_out *= LEG_FAST_HEIGHT_FF_DECAY;
  }

  *ff_out = LIMIT(*ff_out, -LEG_FAST_HEIGHT_FF_LIMIT, LEG_FAST_HEIGHT_FF_LIMIT);
  *last_error = error;

  sp_pid->PID_new_update(mang_pid->OUT_PID, now_sp);

  return LIMIT(sp_pid->OUT_PID + *ff_out, -LEG_FAST_HEIGHT_OUT_LIMIT, LEG_FAST_HEIGHT_OUT_LIMIT);
}
// 按左右镜像方向更新并限制腿长目标角。
static void leg_mang_target_add(f *left_mang_cmd, f *right_mang_cmd, f target_step)
{
  *left_mang_cmd += (LEFT_LEG_MAX_MANG > LEFT_LEG_MIN_MANG) ? target_step : -target_step;
  *right_mang_cmd += (RIGHT_LEG_MAX_MANG > RIGHT_LEG_MIN_MANG) ? target_step : -target_step;
  *left_mang_cmd = LIMIT(*left_mang_cmd,
                         leg_min(LEFT_LEG_MIN_MANG, LEFT_LEG_MAX_MANG),
                         leg_max(LEFT_LEG_MIN_MANG, LEFT_LEG_MAX_MANG));
  *right_mang_cmd = LIMIT(*right_mang_cmd,
                          leg_min(RIGHT_LEG_MIN_MANG, RIGHT_LEG_MAX_MANG),
                          leg_max(RIGHT_LEG_MIN_MANG, RIGHT_LEG_MAX_MANG));
}
// 清空腿长串级 PID 和前馈状态。
static void leg_clear_mang_pid(void)
{
  left_control_mang.Integral = 0;
  left_control_mang.OUT_I = 0;
  left_control_mang.OUT_PID = 0;
  left_control_sp.Integral = 0;
  left_control_sp.OUT_I = 0;
  left_control_sp.OUT_PID = 0;
  right_control_mang.Integral = 0;
  right_control_mang.OUT_I = 0;
  right_control_mang.OUT_PID = 0;
  right_control_sp.Integral = 0;
  right_control_sp.OUT_I = 0;
  right_control_sp.OUT_PID = 0;
  left_mang_ff = 0;
  right_mang_ff = 0;
  left_mang_last_error = 0;
  right_mang_last_error = 0;
}

static void leg_init_height_target(const LegControlInput *input,
                                   f *left_mang_cmd,
                                   f *right_mang_cmd)
{
  *left_mang_cmd = LIMIT(input->left_motor->mang,
                         leg_min(LEFT_LEG_MIN_MANG, LEFT_LEG_MAX_MANG),
                         leg_max(LEFT_LEG_MIN_MANG, LEFT_LEG_MAX_MANG));
  *right_mang_cmd = LIMIT(input->right_motor->mang,
                          leg_min(RIGHT_LEG_MIN_MANG, RIGHT_LEG_MAX_MANG),
                          leg_max(RIGHT_LEG_MIN_MANG, RIGHT_LEG_MAX_MANG));
  leg_clear_mang_pid();
}

static f leg_height_target_step(const LegControlInput *input)
{
  if (input->yk_mode == FAST_CHASSIC)
  {
    if (input->ch3 > 20 || input->ch3 < -20)
      return input->ch3 / (LEG_FAST_HEIGHT_TARGET_STEP_DIV * 10.0f);
    return 0;
  }

  if (input->yk_mode == PLAYER_MODE && leg_control_mode && input->key_e)
  {
    f target_step = input->key_ctrl ? -1.0f : 1.0f;
    return target_step / (LEG_FAST_HEIGHT_TARGET_STEP_DIV / 10.0f);
  }

  return 0;
}

static void leg_height_control_update(const LegControlInput *input,
                                      f *left_mang_cmd,
                                      f *right_mang_cmd)
{
  leg_dm_mit_kd = LEG_DM_MIT_KD;
  leg_roll_cmd = 0;
  leg_pitch_cmd = 0;
  leg_mang_target_add(left_mang_cmd,
                      right_mang_cmd,
                      leg_height_target_step(input));

  leg_output.left_torque = leg_fast_mang_cmd(&left_control_mang,
                                             &left_control_sp,
                                             *left_mang_cmd,
                                             input->left_motor->mang,
                                             input->left_motor->sp,
                                             &left_mang_ff,
                                             &left_mang_last_error);
  leg_output.right_torque = leg_fast_mang_cmd(&right_control_mang,
                                              &right_control_sp,
                                              *right_mang_cmd,
                                              input->right_motor->mang,
                                              input->right_motor->sp,
                                              &right_mang_ff,
                                              &right_mang_last_error);
}

// 配置腿部电机允许的力矩范围。
void Leg_Control_Config_MotorLimit(MOTOR_DM *left_motor, MOTOR_DM *right_motor)
{
  if (left_motor != 0)
  {
    left_motor->T_MAX = 120;
    left_motor->T_MIN = -120;
  }

  if (right_motor != 0)
  {
    right_motor->T_MAX = 120;
    right_motor->T_MIN = -120;
  }
}

uint8_t Leg_Control_Mode_Update(uint8_t player_mode_active, uint8_t x_key_pressed)
{
  if (player_mode_active)
  {
    if (leg_mode_key.updata(x_key_pressed) == UpDown_check_rising)
    {
      leg_control_mode ^= 1;
      return 1;
    }
    return 0;
  }

  leg_mode_key.updata(0);
  if (leg_control_mode)
  {
    leg_control_mode = 0;
    return 1;
  }
  return 0;
}

uint8_t Leg_Control_Mode_Is_Enabled(void)
{
  return leg_control_mode;
}
// 兼容旧调用方；实际控制统一走 SMC 入口。
void Leg_Control_Update(const LegControlInput *input)
{
  Leg_SMC_Control(input);
}

void Leg_Control_Get_Output(LegControlOutput *output)
{
  if (output != 0)
    *output = leg_output;
}

float Leg_Control_Get_Left_Torque(void)
{
  return leg_output.left_torque;
}

float Leg_Control_Get_Right_Torque(void)
{
  return leg_output.right_torque;
}

float Leg_Control_Get_Mit_Kd(void)
{
  return leg_output.mit_kd;
}

float Leg_Control_Get_Roll_Cmd(void)
{
  return leg_output.roll_cmd;
}
void Leg_SMC_Control(const LegControlInput *input)
{
  static f now_left_mang_cmd, now_right_mang_cmd;
  static f leg_smc_last_left_torque, leg_smc_last_right_torque;
  static u8 leg_height_mode_last = 0;
  static u8 leg_smc_mode_last = 0;
  if (input == 0 || input->left_motor == 0 || input->right_motor == 0)
  {
    leg_output.left_torque = 0;
    leg_output.right_torque = 0;
    leg_dm_mit_kd = LEG_DM_MIT_KD;
    leg_roll_cmd = 0;
    leg_pitch_cmd = 0;
    leg_height_mode_last = 0;
    leg_smc_mode_last = 0;
    leg_clear_mang_pid();
    leg_sync_output();
    return;
  }

  u8 leg_height_mode_now = (input->yk_mode == FAST_CHASSIC ||
                            (input->yk_mode == PLAYER_MODE && leg_control_mode));

  if (leg_height_mode_now)
  {
    leg_smc_mode_last = 0;
    if (leg_height_mode_last == 0)
      leg_init_height_target(input, &now_left_mang_cmd, &now_right_mang_cmd);
    leg_height_mode_last = 1;
    leg_height_control_update(input, &now_left_mang_cmd, &now_right_mang_cmd);
  }
  else if (input->yk_mode == CONTROL_MODE || input->yk_mode == PLAYER_MODE ||
           input->yk_mode == XTL_MODE)
  {
    leg_height_mode_last = 0;
    f last_left_torque = leg_output.left_torque;
    f last_right_torque = leg_output.right_torque;
    float roll_smc_iinput = input->gimbal_roll - LEG_ROLL_KEEP_TARGET_ANGLE;
    leg_roll_smc.ref = LEG_ROLL_KEEP_TARGET_ANGLE;
    leg_roll_smc.SMC_Tick(LEG_ROLL_KEEP_TARGET_ANGLE,roll_smc_iinput,0,input->gimbal_roll,input->gimbal_roll_acc);
    leg_roll_cmd = torque_return(leg_roll_smc.u);
    leg_set_balance_output(input, leg_roll_cmd, 0, 40.0f);
    if (leg_smc_mode_last == 0)
    {
      leg_smc_last_left_torque = last_left_torque;
      leg_smc_last_right_torque = last_right_torque;
    }
    leg_smc_last_left_torque += LIMIT(leg_output.left_torque - leg_smc_last_left_torque,
                                      -LEG_SMC_OUTPUT_SLEW_STEP,
                                       LEG_SMC_OUTPUT_SLEW_STEP);
    leg_smc_last_right_torque += LIMIT(leg_output.right_torque - leg_smc_last_right_torque,
                                       -LEG_SMC_OUTPUT_SLEW_STEP,
                                        LEG_SMC_OUTPUT_SLEW_STEP);
    leg_output.left_torque = leg_smc_last_left_torque;
    leg_output.right_torque = leg_smc_last_right_torque;
    leg_smc_mode_last = 1;
  }
  else
  {
    leg_height_mode_last = 0;
    leg_smc_mode_last = 0;
    leg_roll_cmd = 0;
    leg_pitch_cmd = 0;
    leg_output.left_torque = 0;
    leg_output.right_torque = 0;
  }
  leg_sync_output();
}
