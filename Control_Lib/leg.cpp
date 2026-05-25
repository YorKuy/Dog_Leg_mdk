#include "leg.h"
#include "main.h"
#include <math.h>
#include "SMC.h"

typedef float f;
typedef uint8_t u8;

#define LEFT_LEG_MAX_MANG 2.97055F
#define RIGHT_LEG_MAX_MANG -2.58264F
#define LEFT_LEG_MIN_MANG 2.25292F
#define RIGHT_LEG_MIN_MANG -1.85264F

#define LEG_DM_MIT_KD 3.5f
#define LEG_DM_MIT_STATIC_KD 0.6f
#define LEG_JOINT_DAMPING_GAIN 0.60f
#define LEG_JOINT_DAMPING_SPEED_DEADZONE 0.20f
#define LEG_STATIC_DAMPING_GAIN 0.20f
#define LEG_STATIC_DAMPING_SPEED_DEADZONE 0.50f
#define LEG_ACCEL_FF_SIGN 1.0f
#define LEG_ACCEL_FF_GAIN 0.0f
#define LEG_ACCEL_FF_LIMIT 8.0f
#define LEG_ACCEL_FF_DEADZONE_G 0.03f
#define LEG_ACCEL_FF_BIAS_ALPHA 0.002f
#define LEG_ACCEL_FF_OUT_ALPHA 0.25f
#define LEG_PITCH_TARGET_ANGLE 0.0f
#define LEG_PITCH_BALANCE_SIGN -1.0f
#define LEG_PITCH_OUTPUT_LIMIT 18.0f
#define LEG_PITCH_START_ANGLE 5.0f
#define LEG_PITCH_STOP_ANGLE 3.0f
#define LEG_CHASSIS_ACCEL_FF_SIGN 1.0f
#define LEG_CHASSIS_ACCEL_FF_GAIN 1.5f
#define LEG_CHASSIS_BRAKE_FF_GAIN 1.0f
#define LEG_CHASSIS_ACCEL_FF_LIMIT 10.0f
#define LEG_CHASSIS_ACCEL_FF_DEADZONE 1.0f
#define LEG_CHASSIS_ACCEL_FF_ALPHA 0.58f
#define LEG_CHASSIS_ACCEL_FF_SPEED_DEADZONE 10.0f
#define LEG_CHASSIS_ACCEL_FF_STOP_SPEED_DEADZONE 8.0f
#define LEG_CONTROL_LR_ACCEL_FF_SIGN -1.0f
#define LEG_CONTROL_LR_ACCEL_FF_SPEED_DEADZONE 120.0f
#define LEG_CONTROL_LR_BRAKE_FF_GAIN 0.80f
#define LEG_PLAYER_ACCEL_FF_SIGN 1.0f
#define LEG_PLAYER_ACCEL_FF_GAIN 2.50f
#define LEG_PLAYER_BRAKE_FF_GAIN 2.80f
#define LEG_PLAYER_ACCEL_FF_LIMIT 6.0f
#define LEG_PLAYER_ACCEL_FF_ALPHA 0.85f
#define LEG_PLAYER_XTL_ACCEL_FF_GAIN 1.50f
#define LEG_PLAYER_XTL_BRAKE_FF_GAIN 1.90f
#define LEG_PLAYER_XTL_ACCEL_FF_LIMIT 8.0f
#define LEG_PLAYER_XTL_ACCEL_FF_ALPHA 0.30f
#define LEG_PLAYER_STATIC_SPEED_DEADZONE 8.0f
#define LEG_PLAYER_STATIC_ROLL_DEADZONE 0.8f
#define LEG_PLAYER_STATIC_PITCH_DEADZONE 3.0f
#define LEG_PLAYER_STATIC_EXIT_SPEED_DEADZONE 8.0f
#define LEG_PLAYER_STATIC_EXIT_ROLL_DEADZONE 1.6f
#define LEG_PLAYER_STATIC_EXIT_PITCH_DEADZONE 5.0f
#define LEG_PLAYER_STATIC_TORQUE_DEADZONE 0.20f
#define LEG_ROLL_BALANCE_SIGN 1.0f
#define LEG_ROLL_KEEP_TARGET_ANGLE -3.0f
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
#define LEG_DEATH_ROLL_ANGLE_I_LIMIT 150.0f
#define LEG_DEATH_ROLL_SPEED_I_LIMIT 2.0f
#define LEG_DEATH_PITCH_ANGLE_I_LIMIT 150.0f
#define LEG_DEATH_PITCH_SPEED_I_LIMIT 1.5f

static PID_class left_control_mang(120.0f, 0.0f, 0.0f, 84.0f, 0.0f, 0.0f, 84.0f, 0.06f, 0.0f),
    left_control_sp(0.32f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f, 20.0f, 0.20f, 0.0f),
    right_control_mang(120.0f, 0.0f, 0.0f, 84.0f, 0.0f, 0.0f, 84.0f, 0.06f, 0.0f),
    right_control_sp(0.32f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f, 20.0f, 0.20f, 0.0f);

static PID_class leg_imu_mang(43.0f, 0.05f, 75.0f, 900.0f, 150.0f, 200.0f, 700.0f, 2.5f, 1.0f),
    leg_imu_sp(0.050f, 0.00045f, 0.0f, 48.0f, 3.5f, 0.0f, 48.0f, 1.0f, 45.0f),
    leg_imu_pitch_mang(33.0f, 0.08f, 60.0f, 700.0f, 150.0f, 250.0f, 500.0f, 2.0f, 0.8f),
    leg_imu_pitch_sp(0.050f, 0.00040f, 0.0f, 30.0f, 2.0f, 0.0f, 30.0f, 1.0f, 35.0f); 

static SMC_PITCH leg_roll_smc(45,75, 0.15f, 0.001f, 7000, 0.8f, 1.0f),
                 leg_control_left(40,70,0.2f,0.001f,7000,0.8f,1.0f),
                 leg_control_right(40,70,0.2f,0.001f,7000,0.8f,1.0f);
static UpDown_check_class leg_mode_key(0);
static u8 leg_control_mode = 0;
static f leg_target_angel;
static f leg_torque_cmd;
static f leg_compensation_cmd;
static u8 leg_compensation_mode = 0;
static f leg_compensation_out;
static f leg_accel_bias_g = 0;
static f leg_accel_dynamic_g = 0;
static f leg_accel_ff_out = 0;
static u8 leg_accel_ff_inited = 0;
static f leg_chassis_fb_speed_last = 0;
static f leg_chassis_accel_raw = 0;
static f leg_chassis_accel_lpf = 0;
static f leg_chassis_accel_ff_out = 0;
static u8 leg_chassis_accel_ff_inited = 0;
static u8 leg_chassis_xtl_flag_last = 0;
static u8 leg_player_static_active = 0;
static f leg_player_static_hold_torque = 0;
static f leg_dm_mit_kd = LEG_DM_MIT_KD;
static f leg_roll_cmd, leg_pitch_cmd;
static f low_roll_compensation;
static f left_mang_ff, right_mang_ff, left_mang_last_error, right_mang_last_error;
static LegControlOutput leg_output = {0.0f, 0.0f, LEG_DM_MIT_KD, 0.0f, 0.0f};
static float torque_return(float u)//力矩转换
{
    float A = u / (16384.0f / 3.0f);
    float nm = A * 0.741f * 40.0f;
    return nm;
}

static f leg_min(f a, f b)
{
  return (a < b) ? a : b;
}

static f leg_max(f a, f b)
{
  return (a > b) ? a : b;
}

static void leg_sync_output(void) //赋值
{
  leg_output.mit_kd = leg_dm_mit_kd;
  leg_output.roll_cmd = leg_roll_cmd;
  leg_output.pitch_cmd = leg_pitch_cmd;
}
//player模式下的静态补偿相关函数
static u8 leg_player_static_quiet_in_range(const LegControlInput *input,
                                           f speed_deadzone,
                                           f roll_deadzone,
                                           f pitch_deadzone)
{
  return input->yk_mode == PLAYER_MODE &&
         fabsf(input->fb_real_speed) < speed_deadzone &&
         fabsf(input->lr_real_speed) < speed_deadzone &&
         fabsf(input->gimbal_roll - LEG_ROLL_KEEP_TARGET_ANGLE) < roll_deadzone &&
         fabsf(input->gimbal_pitch - LEG_PITCH_TARGET_ANGLE) < pitch_deadzone;
}
//player模式下的静态补偿
static u8 leg_player_static_quiet(const LegControlInput *input)
{
  if (leg_player_static_active)
  {
    if (leg_player_static_quiet_in_range(input,
                                         LEG_PLAYER_STATIC_EXIT_SPEED_DEADZONE,
                                         LEG_PLAYER_STATIC_EXIT_ROLL_DEADZONE,
                                         LEG_PLAYER_STATIC_EXIT_PITCH_DEADZONE))
      return 1;

    leg_player_static_active = 0;
    return 0;
  }

  if (leg_player_static_quiet_in_range(input,
                                       LEG_PLAYER_STATIC_SPEED_DEADZONE,
                                       LEG_PLAYER_STATIC_ROLL_DEADZONE,
                                       LEG_PLAYER_STATIC_PITCH_DEADZONE))
  {
    leg_player_static_active = 1;
    leg_player_static_hold_torque = leg_torque_cmd;
    if (fabsf(leg_player_static_hold_torque) < LEG_PLAYER_STATIC_TORQUE_DEADZONE)
      leg_player_static_hold_torque = 0;
    return 1;
  }
  return 0;
}

static void leg_reset_player_static_quiet(void)
{
  leg_player_static_active = 0;
  leg_player_static_hold_torque = 0;
  leg_dm_mit_kd = LEG_DM_MIT_KD;
}

static void leg_clear_roll_pid(void)
{
  leg_imu_mang.Integral = 0;
  leg_imu_mang.OUT_I = 0;
  leg_imu_mang.OUT_PID = 0;
  leg_imu_sp.Integral = 0;
  leg_imu_sp.OUT_I = 0;
  leg_imu_sp.OUT_PID = 0;
}
//清除横滚PID，重置俯仰平衡状态
static void leg_clear_pitch_pid(void)
{
  leg_imu_pitch_mang.Integral = 0;
  leg_imu_pitch_mang.OUT_I = 0;
  leg_imu_pitch_mang.OUT_PID = 0;
  leg_imu_pitch_sp.Integral = 0;
  leg_imu_pitch_sp.OUT_I = 0;
  leg_imu_pitch_sp.OUT_PID = 0;
}
//重置平衡状态
static void leg_reset_pitch_balance(void)
{
  leg_clear_pitch_pid();
  leg_compensation_mode = 0;
  leg_compensation_out = 0;
}
//死亡检测，防止积分过大导致失控
static u8 leg_integral_death_check(void)
{
  if (fabsf(leg_imu_mang.OUT_I) >= LEG_DEATH_ROLL_ANGLE_I_LIMIT ||
      fabsf(leg_imu_sp.Integral) >= LEG_DEATH_ROLL_SPEED_I_LIMIT ||
      fabsf(leg_imu_pitch_mang.OUT_I) >= LEG_DEATH_PITCH_ANGLE_I_LIMIT ||
      fabsf(leg_imu_pitch_sp.Integral) >= LEG_DEATH_PITCH_SPEED_I_LIMIT)
  {
    leg_clear_roll_pid();
    leg_reset_pitch_balance();
    leg_torque_cmd = 0;
    return 1;
  }
  return 0;
}
//软件关节阻尼，防止震荡
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
//平衡输出，包含补偿和关节阻尼
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

  if (leg_compensation_mode)
  {
    f roll_allow = leg_remaining_output(torque_limit, pitch_out);

    if (fabsf(roll_out) > roll_allow)
    {
      roll_out = LIMIT(roll_out, -roll_allow, roll_allow);
    }
  }
  else
  {
    f pitch_allow = leg_remaining_output(torque_limit, roll_out);
    pitch_out = LIMIT(pitch_out, -pitch_allow, pitch_allow);
  }

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
//设置平衡输出，包含补偿和关节阻尼，使用默认的阻尼参数
static void leg_set_balance_output(const LegControlInput *input, f roll_cmd, f pitch_cmd, f torque_limit)
{
  leg_set_balance_output_custom(input,
                                roll_cmd,
                                pitch_cmd,
                                torque_limit,
                                LEG_JOINT_DAMPING_GAIN,
                                LEG_JOINT_DAMPING_SPEED_DEADZONE);
}
//腿部加速度补偿更新
static f leg_accel_feedforward_update(const LegControlInput *input)
{
  f acc_roll_g = input->gimbal_roll_acc;
  if (leg_accel_ff_inited == 0)
  {
    leg_accel_bias_g = acc_roll_g;
    leg_accel_dynamic_g = 0;
    leg_accel_ff_out = 0;
    leg_accel_ff_inited = 1;
  }

  leg_accel_bias_g += LEG_ACCEL_FF_BIAS_ALPHA * (acc_roll_g - leg_accel_bias_g);
  f acc_dynamic_g = acc_roll_g - leg_accel_bias_g;
  if (fabsf(acc_dynamic_g) < LEG_ACCEL_FF_DEADZONE_G)
    acc_dynamic_g = 0;

  leg_accel_dynamic_g += LEG_ACCEL_FF_OUT_ALPHA * (acc_dynamic_g - leg_accel_dynamic_g);
  leg_accel_ff_out = LIMIT(LEG_ACCEL_FF_SIGN * LEG_ACCEL_FF_GAIN * leg_accel_dynamic_g,
                           -LEG_ACCEL_FF_LIMIT,
                            LEG_ACCEL_FF_LIMIT);
  return leg_accel_ff_out;
}
//获取底盘前后速度
static f leg_get_chassis_fb_speed(const LegControlInput *input)
{
  if (input->yk_mode == PLAYER_MODE)
  {
    f player_fb_speed = input->fb_real_speed;
    f player_lr_speed = input->lr_real_speed;

    if (input->xtl_flag)
    {
      player_fb_speed = input->fb_real_speed * input->cos_theta + input->lr_real_speed * input->sin_theta;
      player_lr_speed = input->lr_real_speed * input->cos_theta - input->fb_real_speed * input->sin_theta;
    }

    if (fabsf(player_lr_speed) > LEG_CONTROL_LR_ACCEL_FF_SPEED_DEADZONE &&
        fabsf(player_fb_speed) < fabsf(player_lr_speed))
    {
      if (player_lr_speed > 0)
        player_lr_speed -= LEG_CONTROL_LR_ACCEL_FF_SPEED_DEADZONE;
      else
        player_lr_speed += LEG_CONTROL_LR_ACCEL_FF_SPEED_DEADZONE;
      return LEG_CONTROL_LR_ACCEL_FF_SIGN * player_lr_speed;
    }

    return player_fb_speed;
  }
  if (input->yk_mode == CONTROL_MODE)
  {
    f lr_speed = input->chassis_ch1_real;
    if (fabsf(lr_speed) < LEG_CONTROL_LR_ACCEL_FF_SPEED_DEADZONE)
      return 0;
    if (lr_speed > 0)
      lr_speed -= LEG_CONTROL_LR_ACCEL_FF_SPEED_DEADZONE;
    else
      lr_speed += LEG_CONTROL_LR_ACCEL_FF_SPEED_DEADZONE;
    return LEG_CONTROL_LR_ACCEL_FF_SIGN * lr_speed;
  }
  if (input->yk_mode == ONLY_CHASSIC || input->yk_mode == FAST_CHASSIC || input->yk_mode == XTL_MODE)
    return input->chassis_ch0_real;
  return 0;
}
//重置底盘加速度补偿状态
static void leg_reset_chassis_accel_ff(const LegControlInput *input)
{
  leg_chassis_fb_speed_last = leg_get_chassis_fb_speed(input);
  leg_chassis_accel_raw = 0;
  leg_chassis_accel_lpf = 0;
  leg_chassis_accel_ff_out = 0;
  leg_chassis_accel_ff_inited = 1;
  leg_chassis_xtl_flag_last = (input->yk_mode == PLAYER_MODE && input->xtl_flag);
}

static f leg_chassis_accel_feedforward_update(const LegControlInput *input)
{
  f chassis_fb_speed = leg_get_chassis_fb_speed(input);
  f chassis_fb_speed_last = leg_chassis_fb_speed_last;
  u8 player_xtl_ff = (input->yk_mode == PLAYER_MODE && input->xtl_flag);

  if (leg_chassis_accel_ff_inited == 0 || player_xtl_ff != leg_chassis_xtl_flag_last)
  {
    leg_chassis_fb_speed_last = chassis_fb_speed;
    leg_chassis_accel_raw = 0;
    leg_chassis_accel_lpf = 0;
    leg_chassis_accel_ff_out = 0;
    leg_chassis_accel_ff_inited = 1;
    leg_chassis_xtl_flag_last = player_xtl_ff;
    return 0;
  }

  if ((input->yk_mode == CONTROL_MODE || input->yk_mode == XTL_MODE) &&
      fabsf(input->chassis_ch1_real) > 80.0f &&
      fabsf(input->chassis_ch0_real) < fabsf(input->chassis_ch1_real))
  {
    leg_chassis_fb_speed_last = chassis_fb_speed;
    leg_chassis_accel_raw = 0;
    leg_chassis_accel_lpf = 0;
    leg_chassis_accel_ff_out = 0;
    return 0;
  }

  leg_chassis_accel_raw = chassis_fb_speed - chassis_fb_speed_last;
  leg_chassis_fb_speed_last = chassis_fb_speed;

  if (fabsf(leg_chassis_accel_raw) < LEG_CHASSIS_ACCEL_FF_DEADZONE)
    leg_chassis_accel_raw = 0;

  if (fabsf(chassis_fb_speed) < LEG_CHASSIS_ACCEL_FF_STOP_SPEED_DEADZONE &&
      leg_chassis_accel_raw == 0)
  {
    leg_chassis_accel_lpf = 0;
    leg_chassis_accel_ff_out = 0;
    return 0;
  }

  f ff_alpha = (input->yk_mode == PLAYER_MODE) ? LEG_PLAYER_ACCEL_FF_ALPHA : LEG_CHASSIS_ACCEL_FF_ALPHA;
  f ff_limit = (input->yk_mode == PLAYER_MODE) ? LEG_PLAYER_ACCEL_FF_LIMIT : LEG_CHASSIS_ACCEL_FF_LIMIT;

  if (player_xtl_ff)
  {
    ff_alpha = LEG_PLAYER_XTL_ACCEL_FF_ALPHA;
    ff_limit = LEG_PLAYER_XTL_ACCEL_FF_LIMIT;
  }

  leg_chassis_accel_lpf += ff_alpha * (leg_chassis_accel_raw - leg_chassis_accel_lpf);

  f ff_gain = (input->yk_mode == PLAYER_MODE) ? LEG_PLAYER_ACCEL_FF_GAIN : LEG_CHASSIS_ACCEL_FF_GAIN;

  if (player_xtl_ff)
    ff_gain = LEG_PLAYER_XTL_ACCEL_FF_GAIN;

  if (fabsf(chassis_fb_speed_last) > LEG_CHASSIS_ACCEL_FF_SPEED_DEADZONE &&
      chassis_fb_speed_last * leg_chassis_accel_raw < 0)
  {
    ff_gain = (input->yk_mode == PLAYER_MODE) ? LEG_PLAYER_BRAKE_FF_GAIN :
                                                LEG_CHASSIS_BRAKE_FF_GAIN;

    if (player_xtl_ff)
      ff_gain = LEG_PLAYER_XTL_BRAKE_FF_GAIN;
  }

  leg_chassis_accel_ff_out =
      LIMIT(LEG_CHASSIS_ACCEL_FF_SIGN * ff_gain * leg_chassis_accel_lpf,
            -ff_limit,
             ff_limit);

  if (input->yk_mode == PLAYER_MODE)
  {
    leg_chassis_accel_ff_out =
        LIMIT(LEG_PLAYER_ACCEL_FF_SIGN * ff_gain * leg_chassis_accel_lpf,
              -ff_limit,
               ff_limit);
  }

  return leg_chassis_accel_ff_out;
}
//腿部横滚平衡更新
static f leg_pitch_balance_update(const LegControlInput *input)
{
  if (input->yk_mode == FAST_CHASSIC || input->yk_mode == CONTROL_MODE ||
      input->yk_mode == PLAYER_MODE || input->yk_mode == XTL_MODE)
  {
    f pitch_error = input->gimbal_pitch - LEG_PITCH_TARGET_ANGLE;

    if (leg_compensation_mode == 0)
    {
      if (fabsf(pitch_error) > LEG_PITCH_START_ANGLE)
        leg_compensation_mode = 1;
    }
    else
    {
      if (fabsf(pitch_error) < LEG_PITCH_STOP_ANGLE)
        leg_compensation_mode = 0;
    }

    if (leg_compensation_mode)
    {
      leg_compensation_cmd = LEG_PITCH_TARGET_ANGLE;

      leg_imu_pitch_mang.PID_update_LP(leg_compensation_cmd, input->gimbal_pitch, 20);
      leg_imu_pitch_sp.PID_new_update(leg_imu_pitch_mang.OUT_PID, input->gimbal_pitch_acc);

      leg_compensation_out += LIMIT(leg_imu_pitch_sp.OUT_PID - leg_compensation_out,
                                    -0.25f,
                                     0.25f);
    }
    else
    {
      leg_imu_pitch_mang.OUT_I *= 0.98f;
      leg_imu_pitch_sp.OUT_I *= 0.98f;

      leg_compensation_out += LIMIT(0.0f - leg_compensation_out,
                                    -0.25f,
                                     0.25f);
    }
  }
  else
  {
    leg_reset_pitch_balance();
  }

  leg_compensation_out = LIMIT(leg_compensation_out,
                               -LEG_PITCH_OUTPUT_LIMIT,
                                LEG_PITCH_OUTPUT_LIMIT);

  return leg_compensation_out;
}
//编码器值模式
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
//腿部编码器值斜坡更新
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
//清除编码器值PID，重置相关状态
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
//配置电机力矩限制
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
void Leg_Control_Update(const LegControlInput *input)
{
  static u8 leg_pid_mode_last = 0;
  static f now_left_mang_cmd, now_right_mang_cmd;
  const f LEG_TARGET_MIN = -4.0f;
  const f LEG_TARGET_MAX = 25.0f;
  u8 leg_pid_mode_now = 0;

  if (input == 0 || input->left_motor == 0 || input->right_motor == 0)
  {
    leg_output.left_torque = 0;
    leg_output.right_torque = 0;
    leg_dm_mit_kd = LEG_DM_MIT_KD;
    leg_roll_cmd = 0;
    leg_pitch_cmd = 0;
    leg_sync_output();
    return;
  }

  if (input->yk_mode == FAST_CHASSIC)
    leg_pid_mode_now = 1;
  else if (input->yk_mode == CONTROL_MODE)
    leg_pid_mode_now = 2;
  else if (input->yk_mode == PLAYER_MODE && leg_control_mode)
    leg_pid_mode_now = 3;

  if (leg_pid_mode_now != leg_pid_mode_last)
  {
    leg_clear_roll_pid();
    leg_reset_pitch_balance();
    leg_reset_chassis_accel_ff(input);
    leg_reset_player_static_quiet();
    leg_torque_cmd = 0;
    if (leg_pid_mode_now == 1 || leg_pid_mode_now == 3)
    {
      leg_target_angel = LEG_ROLL_KEEP_TARGET_ANGLE;
      now_left_mang_cmd = LIMIT(input->left_motor->mang,
                                leg_min(LEFT_LEG_MIN_MANG, LEFT_LEG_MAX_MANG),
                                leg_max(LEFT_LEG_MIN_MANG, LEFT_LEG_MAX_MANG));
      now_right_mang_cmd = LIMIT(input->right_motor->mang,
                                 leg_min(RIGHT_LEG_MIN_MANG, RIGHT_LEG_MAX_MANG),
                                 leg_max(RIGHT_LEG_MIN_MANG, RIGHT_LEG_MAX_MANG));
      leg_clear_mang_pid();
    }
    if (leg_pid_mode_now == 2)
    {
      leg_target_angel = LEG_ROLL_KEEP_TARGET_ANGLE;
    }
    leg_pid_mode_last = leg_pid_mode_now;
  }
  leg_target_angel = LIMIT(leg_target_angel, LEG_TARGET_MIN, LEG_TARGET_MAX);

  if (input->yk_mode == FAST_CHASSIC || (input->yk_mode == PLAYER_MODE && leg_control_mode))
  {
    f target_step = 0;
    if (input->yk_mode == FAST_CHASSIC)
    {
      if (input->ch3 > 20 || input->ch3 < -20)
        target_step = input->ch3 / (LEG_FAST_HEIGHT_TARGET_STEP_DIV * 10.0f);
    }
    else if (input->yk_mode == PLAYER_MODE && leg_control_mode && input->key_e)
    {
      target_step = input->key_ctrl ? -1.0f : 1.0f;
      target_step /= LEG_FAST_HEIGHT_TARGET_STEP_DIV / 10.0f;
    }

    leg_dm_mit_kd = LEG_DM_MIT_KD;
    leg_roll_cmd = 0;
    leg_pitch_cmd = 0;
    leg_mang_target_add(&now_left_mang_cmd, &now_right_mang_cmd, target_step); 
    leg_output.left_torque = leg_fast_mang_cmd(&left_control_mang,
                                               &left_control_sp,
                                               now_left_mang_cmd,
                                               input->left_motor->mang,
                                               input->left_motor->sp,
                                               &left_mang_ff,
                                               &left_mang_last_error);
    leg_output.right_torque = leg_fast_mang_cmd(&right_control_mang,
                                                &right_control_sp,
                                                now_right_mang_cmd,
                                                input->right_motor->mang,
                                                input->right_motor->sp,
                                                &right_mang_ff,
                                                &right_mang_last_error);
  }
  else if (input->yk_mode == CONTROL_MODE || input->yk_mode == PLAYER_MODE || input->yk_mode == XTL_MODE)
  {
    f leg_accel_ff = leg_accel_feedforward_update(input);
    f leg_chassis_accel_ff = leg_chassis_accel_feedforward_update(input);
    if (leg_player_static_quiet(input))
    {
      leg_clear_roll_pid();
      leg_reset_pitch_balance();
      leg_reset_chassis_accel_ff(input);
      leg_dm_mit_kd = LEG_DM_MIT_STATIC_KD;
      leg_torque_cmd = leg_player_static_hold_torque;
      leg_roll_cmd = leg_player_static_hold_torque;
      leg_pitch_cmd = 0;
      leg_set_balance_output_custom(input,
                                    leg_player_static_hold_torque,
                                    0,
                                    30.0f,
                                    LEG_STATIC_DAMPING_GAIN,
                                    LEG_STATIC_DAMPING_SPEED_DEADZONE);
      leg_sync_output();
      return;
    }
    leg_dm_mit_kd = LEG_DM_MIT_KD;
    if (input->gimbal_roll < -3.0f)
    {
      low_roll_compensation = fabsf(input->gimbal_roll * 3.5f);
    }
    else
    {
      low_roll_compensation += 0.1f * (5.0f - low_roll_compensation);
    }
    // leg_target_angel = LEG_ROLL_KEEP_TARGET_ANGLE;
    // leg_imu_mang.PID_update_LP(leg_target_angel, input->gimbal_roll, 20);
    // leg_imu_sp.PID_new_update(leg_imu_mang.OUT_PID, input->gimbal_roll_acc);
    // leg_torque_cmd += LIMIT(leg_imu_sp.OUT_PID - leg_torque_cmd, -1.0f, 1.0f);
    // leg_torque_cmd = LIMIT(leg_torque_cmd, -40.0f, 40.0f);
    // leg_roll_cmd = LIMIT(leg_torque_cmd + low_roll_compensation + leg_accel_ff + leg_chassis_accel_ff, -38.0f, 38.0f);
    // leg_pitch_cmd = LIMIT(leg_pitch_balance_update(input),
    //                       -LEG_PITCH_OUTPUT_LIMIT,
    //                        LEG_PITCH_OUTPUT_LIMIT);
    float roll_smc_iinput = LEG_ROLL_KEEP_TARGET_ANGLE - input->gimbal_roll;
    leg_roll_smc.ref = LEG_ROLL_KEEP_TARGET_ANGLE;
    leg_roll_smc.SMC_Tick(LEG_ROLL_KEEP_TARGET_ANGLE,roll_smc_iinput,0,input->gimbal_roll,input->gimbal_roll_acc);
    leg_roll_cmd = torque_return(leg_roll_smc.u);
    if (leg_integral_death_check())
    {
      leg_output.left_torque = 0;
      leg_output.right_torque = 0;
      leg_sync_output();
      return;
    }
    leg_set_balance_output(input, leg_roll_cmd, leg_pitch_cmd, 38.0f);
  }
  else
  {
    leg_target_angel = input->gimbal_roll;
    leg_reset_player_static_quiet();
    leg_clear_roll_pid();
    leg_reset_pitch_balance();
    leg_reset_chassis_accel_ff(input);
    leg_torque_cmd = 0;
    leg_roll_cmd = 0;
    leg_pitch_cmd = 0;
    leg_output.left_torque = 0;
    leg_output.right_torque = 0;
  }

  leg_sync_output();
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
  if (input == 0 || input->left_motor == 0 || input->right_motor == 0)
  {
    leg_output.left_torque = 0;
    leg_output.right_torque = 0;
    leg_dm_mit_kd = LEG_DM_MIT_KD;
    leg_roll_cmd = 0;
    leg_pitch_cmd = 0;
    leg_sync_output();
    return;
  }

  if(input->yk_mode == CONTROL_MODE || input->yk_mode == PLAYER_MODE || input->yk_mode == XTL_MODE)
  {
    float roll_smc_iinput = input->gimbal_roll - LEG_ROLL_KEEP_TARGET_ANGLE;
    leg_roll_smc.ref = LEG_ROLL_KEEP_TARGET_ANGLE;
    leg_roll_smc.SMC_Tick(LEG_ROLL_KEEP_TARGET_ANGLE,roll_smc_iinput,0,input->gimbal_roll,input->gimbal_roll_acc);
    leg_roll_cmd = torque_return(leg_roll_smc.u);
    leg_set_balance_output(input, leg_roll_cmd, 0, 38.0f);
  }
  else if (input->yk_mode == FAST_CHASSIC || (input->yk_mode == PLAYER_MODE && leg_control_mode))
  {
    f target_step = 0;
    if (input->yk_mode == FAST_CHASSIC)
    {
      if (input->ch3 > 20 || input->ch3 < -20)
        target_step = input->ch3 / (LEG_FAST_HEIGHT_TARGET_STEP_DIV * 10.0f);
    }
    else if (input->yk_mode == PLAYER_MODE && leg_control_mode && input->key_e)
    {
      target_step = input->key_ctrl ? -1.0f : 1.0f;
      target_step /= LEG_FAST_HEIGHT_TARGET_STEP_DIV / 10.0f;
    }

    leg_dm_mit_kd = LEG_DM_MIT_KD;
    leg_roll_cmd = 0;
    leg_pitch_cmd = 0;
    leg_mang_target_add(&now_left_mang_cmd, &now_right_mang_cmd, target_step);
    leg_control_left.SMC_Tick(now_left_mang_cmd,target_step, 0, input->left_motor->mang, input->left_motor->sp);
    leg_control_right.SMC_Tick(now_right_mang_cmd,target_step, 0, input->right_motor->mang, input->right_motor->sp);
    leg_output.left_torque =LIMIT(torque_return(leg_control_left.u), -LEG_FAST_HEIGHT_OUT_LIMIT, LEG_FAST_HEIGHT_OUT_LIMIT);
    leg_output.right_torque = LIMIT(torque_return(leg_control_right.u), -LEG_FAST_HEIGHT_OUT_LIMIT, LEG_FAST_HEIGHT_OUT_LIMIT);
  }
  else
  {
    leg_target_angel = input->gimbal_roll;
    leg_reset_player_static_quiet();
    leg_clear_roll_pid();
    leg_reset_pitch_balance();
    leg_reset_chassis_accel_ff(input);
    leg_torque_cmd = 0;
    leg_roll_cmd = 0;
    leg_pitch_cmd = 0;
    leg_output.left_torque = 0;
    leg_output.right_torque = 0;
  }
  leg_sync_output();
}