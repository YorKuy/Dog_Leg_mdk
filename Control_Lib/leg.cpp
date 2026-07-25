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
#define LEG_ROLL_KEEP_TARGET_ANGLE 1.0f
#define LEG_SMC_OUTPUT_SLEW_STEP 1.25f

// Dynamic compensation parameters. The active loop runs at 1 kHz.
#define LEG_CONTROL_DT_DEFAULT 0.001f
#define LEG_ACCEL_FF_ACCEL_TIME_CONSTANT 0.020f
#define LEG_ACCEL_FF_BRAKE_TIME_CONSTANT 0.008f
#define LEG_ACCEL_FF_DECAY_TIME 0.030f
#define LEG_DYNAMIC_AUX_LIMIT 8.0f
#define LEG_HEIGHT_DAMPING_LIMIT 5.0f
#define LEG_HEIGHT_SPEED_FILTER_TIME_CONSTANT 0.016f
// Verified mechanism relation: height increasing drives physical pitch negative.
// A positive roll command lowers the leg, so this sign produces true damping.
#define LEG_HEIGHT_DAMPING_TO_ROLL_SIGN 1.0f

// Physical roll correction. Positive command retracts the left leg; negative
// command retracts the right leg. Only the selected side receives correction.
#define LEG_ROLL_BALANCE_FILTER_TIME_CONSTANT 0.010f
#define LEG_ROLL_RATE_FILTER_TIME_CONSTANT 0.020f
#define LEG_ROLL_RETRACT_SOFT_ZONE 0.05f

// Forward slope/support detection and one-way height hold.
#define LEG_SLOPE_FORWARD_ENTER 120.0f
#define LEG_SLOPE_FORWARD_EXIT 60.0f
#define LEG_SLOPE_PITCH_ENTER 2.0f
#define LEG_SLOPE_STALL_ENTER 0.20f
#define LEG_SLOPE_SMC_ENTER 6.0f
#define LEG_SLOPE_CONFIRM_TIME 0.032f
#define LEG_SLOPE_CANDIDATE_TIMEOUT 0.300f
#define LEG_SUPPORT_TIMEOUT 2.500f
#define LEG_HEIGHT_HOLD_DEADZONE 0.005f
#define LEG_HEIGHT_REF_RELEASE_RATE 0.01f
#define LEG_HEIGHT_TO_ROLL_SIGN -1.0f
#define LEG_HEIGHT_EMERGENCY_PITCH 8.0f
#define LEG_HEIGHT_EMERGENCY_RATE 60.0f

// Rear-wheel unloading catch.
#define LEG_UNLOAD_HEIGHT_SPEED -0.25f
#define LEG_UNLOAD_TORQUE_DROP_RATIO 0.70f
#define LEG_UNLOAD_PITCH_RATE 15.0f
#define LEG_UNLOAD_CONFIRM_TIME 0.008f
#define LEG_UNLOAD_MIN_TIME 0.080f
#define LEG_UNLOAD_MAX_TIME 0.200f
#define LEG_UNLOAD_TORQUE_LIMIT 30.0f
#define LEG_UNLOAD_RETRACT_SLEW_PER_STEP 0.30f
#define LEG_UNLOAD_BRAKE_SLEW_PER_STEP 0.80f
#define LEG_UNLOAD_DAMPING_GAIN 1.80f
#define LEG_UNLOAD_MIT_KD 4.20f
#define LEG_SETTLE_TIME 0.250f

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

static SMC_PITCH leg_roll_smc(48, 65, 1.05f, 1.5f, 8000, 0.8f, 1.0f);
static UpDown_check_class leg_mode_key(0);
static u8 leg_control_mode = 0;
static f leg_dm_mit_kd = LEG_DM_MIT_KD;
static f leg_roll_cmd;
static f leg_pitch_cmd;
static f left_mang_ff;
static f right_mang_ff;
static f left_mang_last_error;
static f right_mang_last_error;
static LegControlOutput leg_output = {0};

volatile uint8_t leg_enable_accel_feedforward = 1;
volatile uint8_t leg_enable_forward_jerk_limit = 0;
volatile uint8_t leg_enable_height_damping = 1;
volatile uint8_t leg_enable_slope_hold = 0;
volatile uint8_t leg_enable_unload_catch = 0;
volatile uint8_t leg_enable_roll_balance = 1;
volatile float leg_accel_ff_accel_gain = 2.50f;
volatile float leg_accel_ff_brake_gain = 2.80f;
volatile float leg_accel_ff_limit = 6.0f;
volatile float leg_height_damping_gain = 3.5f;
volatile float leg_height_hold_kp = 60.0f;
volatile float leg_height_hold_kd = 3.0f;
volatile float leg_height_hold_limit = 6.0f;
volatile float leg_roll_balance_kp = 1.50f;
volatile float leg_roll_balance_kd = 0.10f;
volatile float leg_roll_balance_limit = 10.0f;
volatile float leg_roll_balance_direction = 1.0f;
volatile uint32_t leg_dm_pair_send_ok_count;
volatile uint32_t leg_dm_pair_send_fail_count;
volatile uint16_t leg_dm_pair_send_consecutive_fail;

static LegDynamicState leg_dynamic_state = LEG_DYNAMIC_NORMAL;
static f leg_dynamic_state_time;
static f leg_slope_confirm_time;
static f leg_unload_confirm_time;
static f leg_height_reference;
static f leg_height_last;
static f leg_height_speed_filtered;
static f leg_feedback_torque_peak;
static f leg_accel_ff_filtered;
static f leg_last_forward_cmd;
static f leg_forward_motion_sign;
static f leg_roll_rate_filtered;
static f leg_roll_balance_filtered;
static u8 leg_height_initialized;
static u8 leg_forward_initialized;

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

static f leg_valid_dt(f dt)
{
  if (dt < 0.0005f || dt > 0.02f)
    return LEG_CONTROL_DT_DEFAULT;
  return dt;
}

static f leg_normalized_height(f angle, f min_angle, f max_angle)
{
  f range = max_angle - min_angle;
  if (fabsf(range) < 1e-6f)
    return 0.0f;
  return LIMIT((angle - min_angle) / range, 0.0f, 1.0f);
}

static void leg_update_height(const LegControlInput *input, f dt)
{
  f left_height = leg_normalized_height(input->left_motor->mang,
                                        LEFT_LEG_MIN_MANG, LEFT_LEG_MAX_MANG);
  f right_height = leg_normalized_height(input->right_motor->mang,
                                         RIGHT_LEG_MIN_MANG, RIGHT_LEG_MAX_MANG);
  leg_output.height = 0.5f * (left_height + right_height);

  if (!leg_height_initialized)
  {
    leg_height_last = leg_output.height;
    leg_height_reference = leg_output.height;
    leg_height_speed_filtered = 0.0f;
    leg_height_initialized = 1;
  }

  f derived_speed = (leg_output.height - leg_height_last) / dt;
  f left_speed = input->left_motor->sp / (LEFT_LEG_MAX_MANG - LEFT_LEG_MIN_MANG);
  f right_speed = input->right_motor->sp / (RIGHT_LEG_MAX_MANG - RIGHT_LEG_MIN_MANG);
  f raw_height_speed = 0.3f * derived_speed +
                       0.7f * 0.5f * (left_speed + right_speed);
  f speed_alpha = dt / (LEG_HEIGHT_SPEED_FILTER_TIME_CONSTANT + dt);
  leg_height_speed_filtered += speed_alpha *
                               (raw_height_speed - leg_height_speed_filtered);
  leg_output.height_speed = leg_height_speed_filtered;
  leg_height_last = leg_output.height;
}

static void leg_enter_dynamic_state(LegDynamicState state)
{
  leg_dynamic_state = state;
  leg_dynamic_state_time = 0.0f;
  leg_slope_confirm_time = 0.0f;
  leg_unload_confirm_time = 0.0f;

  if (state == LEG_DYNAMIC_SUPPORT_HOLD || state == LEG_DYNAMIC_SETTLE)
    leg_height_reference = leg_output.height;
  if (state == LEG_DYNAMIC_SUPPORT_HOLD)
    leg_feedback_torque_peak = 0.0f;
}

static void leg_update_dynamic_state(const LegControlInput *input, f smc_cmd, f dt)
{
  leg_dynamic_state_time += dt;

  if (!leg_enable_slope_hold || input->forward_cmd < LEG_SLOPE_FORWARD_EXIT)
  {
    if (leg_dynamic_state != LEG_DYNAMIC_UNLOAD_CATCH)
      leg_enter_dynamic_state(LEG_DYNAMIC_NORMAL);
  }

  switch (leg_dynamic_state)
  {
    case LEG_DYNAMIC_NORMAL:
      if (leg_enable_slope_hold && input->pitch_data_age <= 0.080f &&
          input->yk_mode != XTL_MODE && !input->xtl_flag &&
          input->forward_cmd > LEG_SLOPE_FORWARD_ENTER &&
          fabsf(input->gimbal_roll - LEG_ROLL_KEEP_TARGET_ANGLE) > LEG_SLOPE_PITCH_ENTER &&
          input->wheel_stall_ratio > LEG_SLOPE_STALL_ENTER &&
          fabsf(smc_cmd) > LEG_SLOPE_SMC_ENTER)
        leg_enter_dynamic_state(LEG_DYNAMIC_SLOPE_CANDIDATE);
      break;

    case LEG_DYNAMIC_SLOPE_CANDIDATE:
      if (input->pitch_data_age > 0.080f || input->yk_mode == XTL_MODE ||
          input->xtl_flag || input->forward_cmd < LEG_SLOPE_FORWARD_EXIT ||
          leg_dynamic_state_time > LEG_SLOPE_CANDIDATE_TIMEOUT)
      {
        leg_enter_dynamic_state(LEG_DYNAMIC_NORMAL);
        break;
      }
      if (fabsf(input->gimbal_roll - LEG_ROLL_KEEP_TARGET_ANGLE) > LEG_SLOPE_PITCH_ENTER &&
          input->wheel_stall_ratio > LEG_SLOPE_STALL_ENTER &&
          fabsf(smc_cmd) > LEG_SLOPE_SMC_ENTER)
      {
        leg_slope_confirm_time += dt;
        if (leg_slope_confirm_time >= LEG_SLOPE_CONFIRM_TIME)
          leg_enter_dynamic_state(LEG_DYNAMIC_SUPPORT_HOLD);
      }
      else
      {
        leg_slope_confirm_time = 0.0f;
      }
      break;

    case LEG_DYNAMIC_SUPPORT_HOLD:
    {
      if (leg_dynamic_state_time > LEG_SUPPORT_TIMEOUT ||
          input->forward_cmd < LEG_SLOPE_FORWARD_EXIT)
      {
        leg_enter_dynamic_state(LEG_DYNAMIC_SETTLE);
        break;
      }

      f feedback_torque = 0.5f *
          (fabsf(input->left_motor->Torque) + fabsf(input->right_motor->Torque));
      if (feedback_torque > leg_feedback_torque_peak)
        leg_feedback_torque_peak = feedback_torque;

      u8 torque_dropped = leg_feedback_torque_peak > 2.0f &&
                          feedback_torque < leg_feedback_torque_peak *
                                                LEG_UNLOAD_TORQUE_DROP_RATIO;
      u8 unload = leg_output.height_speed < LEG_UNLOAD_HEIGHT_SPEED &&
                  (torque_dropped ||
                   fabsf(input->gimbal_roll_acc) > LEG_UNLOAD_PITCH_RATE);
      if (leg_enable_unload_catch && unload)
      {
        leg_unload_confirm_time += dt;
        if (leg_unload_confirm_time >= LEG_UNLOAD_CONFIRM_TIME)
          leg_enter_dynamic_state(LEG_DYNAMIC_UNLOAD_CATCH);
      }
      else
      {
        leg_unload_confirm_time = 0.0f;
      }
      break;
    }

    case LEG_DYNAMIC_UNLOAD_CATCH:
      if (!leg_enable_unload_catch || leg_dynamic_state_time >= LEG_UNLOAD_MAX_TIME ||
          (leg_dynamic_state_time >= LEG_UNLOAD_MIN_TIME &&
           fabsf(leg_output.height_speed) < 0.08f))
        leg_enter_dynamic_state(LEG_DYNAMIC_SETTLE);
      break;

    case LEG_DYNAMIC_SETTLE:
      if (leg_dynamic_state_time >= LEG_SETTLE_TIME)
        leg_enter_dynamic_state(LEG_DYNAMIC_NORMAL);
      break;

    default:
      leg_enter_dynamic_state(LEG_DYNAMIC_NORMAL);
      break;
  }
}

static f leg_accel_feedforward_update(const LegControlInput *input, f dt)
{
  f target = 0.0f;
  u8 braking = 0;
  if (!leg_forward_initialized)
  {
    leg_last_forward_cmd = input->forward_cmd;
    if (fabsf(input->forward_cmd) > 10.0f)
      leg_forward_motion_sign = (input->forward_cmd > 0.0f) ? 1.0f : -1.0f;
    leg_forward_initialized = 1;
  }

  if (leg_enable_accel_feedforward && input->pitch_data_age <= 0.080f)
  {
    f acceleration = input->forward_accel;
    if (!isfinite(acceleration))
      acceleration = input->forward_cmd - leg_last_forward_cmd;
    if (fabsf(input->forward_cmd) > 10.0f &&
        input->forward_cmd * acceleration >= 0.0f)
      leg_forward_motion_sign = (input->forward_cmd > 0.0f) ? 1.0f : -1.0f;

    braking = fabsf(acceleration) > 0.01f &&
              leg_forward_motion_sign * acceleration < 0.0f;
    f gain = braking ? leg_accel_ff_brake_gain : leg_accel_ff_accel_gain;
    target = LIMIT(gain * acceleration, -leg_accel_ff_limit, leg_accel_ff_limit);
  }

  leg_last_forward_cmd = input->forward_cmd;
  f tau = LEG_ACCEL_FF_DECAY_TIME;
  if (leg_enable_accel_feedforward && input->pitch_data_age <= 0.080f)
    tau = braking ? LEG_ACCEL_FF_BRAKE_TIME_CONSTANT :
                    LEG_ACCEL_FF_ACCEL_TIME_CONSTANT;
  f alpha = dt / (tau + dt);
  leg_accel_ff_filtered += alpha * (target - leg_accel_ff_filtered);
  return leg_accel_ff_filtered;
}

static f leg_height_damping_update(void)
{
  if (!leg_enable_height_damping)
    return 0.0f;
  return LIMIT(LEG_HEIGHT_DAMPING_TO_ROLL_SIGN * leg_height_damping_gain *
                   leg_output.height_speed,
               -LEG_HEIGHT_DAMPING_LIMIT, LEG_HEIGHT_DAMPING_LIMIT);
}

static f leg_roll_balance_update(const LegControlInput *input, f dt)
{
  f target_rate = (input->roll_data_age <= 0.080f) ?
                      input->gimbal_pitch_acc : 0.0f;
  f rate_alpha = dt / (LEG_ROLL_RATE_FILTER_TIME_CONSTANT + dt);
  leg_roll_rate_filtered += rate_alpha *
                            (target_rate - leg_roll_rate_filtered);

  f target_cmd = 0.0f;
  if (leg_enable_roll_balance && input->roll_data_age <= 0.080f)
  {
    target_cmd = leg_roll_balance_direction *
                 (leg_roll_balance_kp *
                      (input->gimbal_pitch - LEG_PITCH_TARGET_ANGLE) +
                  leg_roll_balance_kd * leg_roll_rate_filtered);
    target_cmd = LIMIT(target_cmd, -leg_roll_balance_limit,
                                   leg_roll_balance_limit);
  }

  f cmd_alpha = dt / (LEG_ROLL_BALANCE_FILTER_TIME_CONSTANT + dt);
  leg_roll_balance_filtered += cmd_alpha *
                               (target_cmd - leg_roll_balance_filtered);
  return leg_roll_balance_filtered;
}

static f leg_retract_soft_scale(f angle, f min_angle, f max_angle)
{
  f height = leg_normalized_height(angle, min_angle, max_angle);
  return LIMIT(height / LEG_ROLL_RETRACT_SOFT_ZONE, 0.0f, 1.0f);
}

static void leg_apply_roll_retract_only(const LegControlInput *input,
                                        f roll_balance_cmd,
                                        f torque_limit)
{
  if (roll_balance_cmd > 0.0f)
  {
    // Positive physical roll: retract the left leg only.
    f scale = leg_retract_soft_scale(input->left_motor->mang,
                                     LEFT_LEG_MIN_MANG, LEFT_LEG_MAX_MANG);
    leg_output.left_torque = LIMIT(leg_output.left_torque -
                                       roll_balance_cmd * scale,
                                   -torque_limit, torque_limit);
  }
  else if (roll_balance_cmd < 0.0f)
  {
    // Negative physical roll: retract the right mirrored leg only.
    f scale = leg_retract_soft_scale(input->right_motor->mang,
                                     RIGHT_LEG_MIN_MANG, RIGHT_LEG_MAX_MANG);
    leg_output.right_torque = LIMIT(leg_output.right_torque -
                                        roll_balance_cmd * scale,
                                    -torque_limit, torque_limit);
  }
}

static f leg_height_hold_update(const LegControlInput *input, f dt)
{
  if (!leg_enable_slope_hold || leg_dynamic_state != LEG_DYNAMIC_SUPPORT_HOLD ||
      fabsf(input->gimbal_roll - LEG_ROLL_KEEP_TARGET_ANGLE) > LEG_HEIGHT_EMERGENCY_PITCH ||
      fabsf(input->gimbal_roll_acc) > LEG_HEIGHT_EMERGENCY_RATE)
    return 0.0f;

  if (leg_output.height > leg_height_reference)
    leg_height_reference = leg_output.height;
  else
    leg_height_reference = leg_max(leg_output.height,
                                   leg_height_reference -
                                       LEG_HEIGHT_REF_RELEASE_RATE * dt);

  f error = leg_height_reference - leg_output.height - LEG_HEIGHT_HOLD_DEADZONE;
  if (error < 0.0f)
    error = 0.0f;
  f retract_speed = (leg_output.height_speed < 0.0f) ?
                        -leg_output.height_speed : 0.0f;
  f extension_cmd = leg_height_hold_kp * error +
                    leg_height_hold_kd * retract_speed;
  return LEG_HEIGHT_TO_ROLL_SIGN *
         LIMIT(extension_cmd, 0.0f, leg_height_hold_limit);
}

static void leg_sync_output(void)
{
  leg_output.mit_kd = leg_dm_mit_kd;
  leg_output.roll_cmd = leg_roll_cmd;
  leg_output.pitch_cmd = leg_pitch_cmd;
  leg_output.dynamic_state = (uint8_t)leg_dynamic_state;
}

static f leg_limit_retract_torque(f torque, f angle_range, f limit)
{
  if (torque * angle_range >= 0.0f)
    return torque;
  return LIMIT(torque, -limit, limit);
}

static f leg_unload_slew(f target, f last, f angle_range)
{
  f target_motion = target * angle_range;
  f last_motion = last * angle_range;
  f step = LEG_SMC_OUTPUT_SLEW_STEP;

  if (target_motion < last_motion && target_motion < 0.0f)
    step = LEG_UNLOAD_RETRACT_SLEW_PER_STEP;
  else if (target_motion > last_motion && last_motion < 0.0f)
    step = LEG_UNLOAD_BRAKE_SLEW_PER_STEP;

  return last + LIMIT(target - last, -step, step);
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

void Leg_Control_Report_OutputPairResult(uint8_t success)
{
  if (success)
  {
    leg_dm_pair_send_ok_count++;
    leg_dm_pair_send_consecutive_fail = 0;
  }
  else
  {
    leg_dm_pair_send_fail_count++;
    if (leg_dm_pair_send_consecutive_fail < 0xFFFFU)
      leg_dm_pair_send_consecutive_fail++;
  }
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
    leg_height_initialized = 0;
    leg_forward_initialized = 0;
    leg_roll_rate_filtered = 0.0f;
    leg_roll_balance_filtered = 0.0f;
    leg_enter_dynamic_state(LEG_DYNAMIC_NORMAL);
    leg_clear_mang_pid();
    leg_sync_output();
    return;
  }

  f dt = leg_valid_dt(input->dt);
  leg_update_height(input, dt);
  leg_output.smc_cmd = 0.0f;
  leg_output.accel_ff_cmd = 0.0f;
  leg_output.height_damping_cmd = 0.0f;
  leg_output.height_hold_cmd = 0.0f;
  leg_output.roll_balance_cmd = 0.0f;

  u8 leg_height_mode_now = (input->yk_mode == FAST_CHASSIC ||
                            (input->yk_mode == PLAYER_MODE && leg_control_mode));

  if (leg_height_mode_now)
  {
    leg_smc_mode_last = 0;
    leg_enter_dynamic_state(LEG_DYNAMIC_NORMAL);
    leg_accel_ff_filtered = 0.0f;
    leg_forward_initialized = 0;
    leg_roll_rate_filtered = 0.0f;
    leg_roll_balance_filtered = 0.0f;
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
    leg_output.smc_cmd = torque_return(leg_roll_smc.u);
    leg_update_dynamic_state(input, leg_output.smc_cmd, dt);
    leg_output.accel_ff_cmd = leg_accel_feedforward_update(input, dt);
    leg_output.height_damping_cmd = leg_height_damping_update();
    leg_output.height_hold_cmd = leg_height_hold_update(input, dt);
    leg_output.roll_balance_cmd = leg_roll_balance_update(input, dt);

    f auxiliary_cmd = LIMIT(leg_output.accel_ff_cmd +
                                leg_output.height_damping_cmd +
                                leg_output.height_hold_cmd,
                            -LEG_DYNAMIC_AUX_LIMIT, LEG_DYNAMIC_AUX_LIMIT);
    leg_roll_cmd = leg_output.smc_cmd + auxiliary_cmd;
    leg_pitch_cmd = 0.0f;

    f damping_gain = LEG_JOINT_DAMPING_GAIN;
    f torque_limit = 40.0f;
    if (leg_enable_unload_catch &&
        leg_dynamic_state == LEG_DYNAMIC_UNLOAD_CATCH)
    {
      damping_gain = LEG_UNLOAD_DAMPING_GAIN;
      leg_dm_mit_kd = LEG_UNLOAD_MIT_KD;
    }
    else
    {
      leg_dm_mit_kd = LEG_DM_MIT_KD;
    }

    leg_set_balance_output_custom(input, leg_roll_cmd, leg_pitch_cmd, torque_limit,
                                  damping_gain,
                                  LEG_JOINT_DAMPING_SPEED_DEADZONE);
    leg_apply_roll_retract_only(input, leg_output.roll_balance_cmd, torque_limit);

    if (leg_enable_unload_catch &&
        leg_dynamic_state == LEG_DYNAMIC_UNLOAD_CATCH)
    {
      leg_output.left_torque = leg_limit_retract_torque(
          leg_output.left_torque,
          LEFT_LEG_MAX_MANG - LEFT_LEG_MIN_MANG,
          LEG_UNLOAD_TORQUE_LIMIT);
      leg_output.right_torque = leg_limit_retract_torque(
          leg_output.right_torque,
          RIGHT_LEG_MAX_MANG - RIGHT_LEG_MIN_MANG,
          LEG_UNLOAD_TORQUE_LIMIT);
    }
    if (leg_smc_mode_last == 0)
    {
      leg_smc_last_left_torque = last_left_torque;
      leg_smc_last_right_torque = last_right_torque;
    }
    if (leg_enable_unload_catch &&
        leg_dynamic_state == LEG_DYNAMIC_UNLOAD_CATCH)
    {
      leg_smc_last_left_torque = leg_unload_slew(
          leg_output.left_torque, leg_smc_last_left_torque,
          LEFT_LEG_MAX_MANG - LEFT_LEG_MIN_MANG);
      leg_smc_last_right_torque = leg_unload_slew(
          leg_output.right_torque, leg_smc_last_right_torque,
          RIGHT_LEG_MAX_MANG - RIGHT_LEG_MIN_MANG);
    }
    else
    {
      leg_smc_last_left_torque += LIMIT(leg_output.left_torque - leg_smc_last_left_torque,
                                        -LEG_SMC_OUTPUT_SLEW_STEP,
                                         LEG_SMC_OUTPUT_SLEW_STEP);
      leg_smc_last_right_torque += LIMIT(leg_output.right_torque - leg_smc_last_right_torque,
                                         -LEG_SMC_OUTPUT_SLEW_STEP,
                                          LEG_SMC_OUTPUT_SLEW_STEP);
    }
    leg_output.left_torque = leg_smc_last_left_torque;
    leg_output.right_torque = leg_smc_last_right_torque;
    leg_smc_mode_last = 1;
  }
  else
  {
    leg_height_mode_last = 0;
    leg_smc_mode_last = 0;
    leg_enter_dynamic_state(LEG_DYNAMIC_NORMAL);
    leg_accel_ff_filtered = 0.0f;
    leg_forward_initialized = 0;
    leg_roll_rate_filtered = 0.0f;
    leg_roll_balance_filtered = 0.0f;
    leg_roll_cmd = 0;
    leg_pitch_cmd = 0;
    leg_output.left_torque = 0;
    leg_output.right_torque = 0;
  }
  leg_sync_output();
}
