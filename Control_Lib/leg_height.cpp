#include "leg_internal.h"

static PID_class left_control_mang(120.0f, 0.0f, 0.0f, 84.0f, 0.0f, 0.0f, 84.0f, 0.06f, 0.0f),
    left_control_sp(0.32f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f, 20.0f, 0.20f, 0.0f),
    right_control_mang(120.0f, 0.0f, 0.0f, 84.0f, 0.0f, 0.0f, 84.0f, 0.06f, 0.0f),
    right_control_sp(0.32f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f, 20.0f, 0.20f, 0.0f);

static f left_mang_ff;
static f right_mang_ff;
static f left_mang_last_error;
static f right_mang_last_error;

// 腿长角度外环、速度内环及误差增长前馈。
f leg_fast_mang_cmd(PID_class *mang_pid,
                           PID_class *sp_pid,
                           f target_mang,
                           f now_mang,
                           f now_sp,
                           f *ff_out,
                           f *last_error,
                           f dt)
{
  f error = target_mang - now_mang;
  f abs_error = fabsf(error);
  f abs_last_error = fabsf(*last_error);

  mang_pid->PID_new_update(target_mang, now_mang);

  if (abs_error < LEG_FAST_HEIGHT_ERROR_DEADZONE)
  {
    *ff_out = leg_decay_to_zero(*ff_out, dt,
                                LEG_FAST_HEIGHT_FF_REVERSE_TIME);
  }
  else if (error * (*last_error) < 0)
  {
    *ff_out = leg_decay_to_zero(*ff_out, dt,
                                LEG_FAST_HEIGHT_FF_REVERSE_TIME);
  }
  else if (abs_error > abs_last_error + LEG_FAST_HEIGHT_ERROR_GROW_DEADZONE ||
           fabsf(mang_pid->OUT_PID) > LEG_FAST_HEIGHT_LIMIT * LEG_FAST_HEIGHT_PID_FULL_RATIO)
  {
    *ff_out += ((error > 0) ? 1.0f : -1.0f) *
               LEG_FAST_HEIGHT_FF_BUILD_RATE * dt;
  }
  else
  {
    *ff_out = leg_decay_to_zero(*ff_out, dt,
                                LEG_FAST_HEIGHT_FF_DECAY_TIME);
  }

  *ff_out = LIMIT(*ff_out, -LEG_FAST_HEIGHT_FF_LIMIT, LEG_FAST_HEIGHT_FF_LIMIT);
  *last_error = error;

  sp_pid->PID_new_update(mang_pid->OUT_PID, now_sp);

  return LIMIT(sp_pid->OUT_PID + *ff_out, -LEG_FAST_HEIGHT_OUT_LIMIT, LEG_FAST_HEIGHT_OUT_LIMIT);
}
// 按左右镜像方向更新并限制腿长目标角。
void leg_mang_target_add(f *left_mang_cmd, f *right_mang_cmd, f target_step)
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
void leg_clear_mang_pid(void)
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

void leg_init_height_target(const LegControlInput *input,
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

f leg_height_target_step(const LegControlInput *input)
{
  if (input->yk_mode == FAST_CHASSIC)
  {
    if (input->ch3 > 20 || input->ch3 < -20)
      return input->ch3 / (LEG_FAST_HEIGHT_TARGET_STEP_DIV * 10.0f);
    return 0;
  }

  if (input->yk_mode == PLAYER_MODE &&
      Leg_Control_Mode_Is_Enabled() && input->key_e)
  {
    f target_step = input->key_ctrl ? -1.0f : 1.0f;
    return target_step / (LEG_FAST_HEIGHT_TARGET_STEP_DIV / 10.0f);
  }

  return 0;
}

void leg_height_control_update(const LegControlInput *input,
                                      f *left_mang_cmd,
                                      f *right_mang_cmd,
                                      f dt)
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
                                             &left_mang_last_error,
                                             dt);
  leg_output.right_torque = leg_fast_mang_cmd(&right_control_mang,
                                              &right_control_sp,
                                              *right_mang_cmd,
                                              input->right_motor->mang,
                                              input->right_motor->sp,
                                              &right_mang_ff,
                                              &right_mang_last_error,
                                              dt);

  // 腿长保持模式也需要气弹簧方向相关阻尼，否则低速静摩擦释放后仅靠
  // 串级 P 控制容易在目标角附近反复越过目标。
  f damping_gain = LIMIT(leg_joint_damping_gain, 0.0f, 3.0f);
  f damping_deadzone = LIMIT(leg_joint_damping_speed_deadzone,
                             0.0f, 0.50f);
  leg_output.left_torque = leg_apply_joint_damping_custom(
      leg_output.left_torque, input->left_motor, LEG_FAST_HEIGHT_OUT_LIMIT,
      damping_gain, damping_deadzone,
      LEFT_LEG_MAX_MANG - LEFT_LEG_MIN_MANG);
  leg_output.right_torque = leg_apply_joint_damping_custom(
      leg_output.right_torque, input->right_motor, LEG_FAST_HEIGHT_OUT_LIMIT,
      damping_gain, damping_deadzone,
      RIGHT_LEG_MAX_MANG - RIGHT_LEG_MIN_MANG);
}

