#include "leg_internal.h"

// 将 SMC 原始输出换算为电机力矩。
f torque_return(f u)
{
  f current = u / (16384.0f / 3.0f);
  return current * 0.741f * 40.0f;
}

f leg_min(f a, f b)
{
  return (a < b) ? a : b;
}

f leg_max(f a, f b)
{
  return (a > b) ? a : b;
}

f leg_valid_dt(f dt)
{
  if (dt < 0.0005f || dt > 0.02f)
    return LEG_CONTROL_DT_DEFAULT;
  return dt;
}

f leg_decay_to_zero(f value, f dt, f time_constant)
{
  f tau = (time_constant > dt) ? time_constant : dt;
  return value * (tau / (tau + dt));
}

f leg_normalized_height(f angle, f min_angle, f max_angle)
{
  f range = max_angle - min_angle;
  if (fabsf(range) < 1e-6f)
    return 0.0f;
  return LIMIT((angle - min_angle) / range, 0.0f, 1.0f);
}

void leg_update_height(const LegControlInput *input, f dt)
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
  f speed_filter_time = LIMIT(leg_height_speed_filter_time, 0.002f, 0.100f);
  f speed_alpha = dt / (speed_filter_time + dt);
  leg_height_speed_filtered += speed_alpha *
                               (raw_height_speed - leg_height_speed_filtered);
  leg_output.height_speed = leg_height_speed_filtered;
  leg_height_last = leg_output.height;
}

void leg_enter_dynamic_state(LegDynamicState state)
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

void leg_update_dynamic_state(const LegControlInput *input, f smc_cmd, f dt)
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

f leg_accel_feedforward_update(const LegControlInput *input, f dt)
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

f leg_height_damping_update(void)
{
  if (!leg_enable_height_damping)
    return 0.0f;
  return LIMIT(LEG_HEIGHT_DAMPING_TO_ROLL_SIGN * leg_height_damping_gain *
                   leg_output.height_speed,
               -LEG_HEIGHT_DAMPING_LIMIT, LEG_HEIGHT_DAMPING_LIMIT);
}

f leg_roll_balance_update(const LegControlInput *input, f dt)
{
  f target_rate = (input->roll_data_age <= 0.080f) ?
                      input->gimbal_pitch_acc : 0.0f;
  f rate_alpha = dt / (LEG_ROLL_RATE_FILTER_TIME_CONSTANT + dt);
  leg_roll_rate_filtered += rate_alpha *
                            (target_rate - leg_roll_rate_filtered);

  f target_cmd = 0.0f;
  if (leg_enable_roll_balance && input->roll_data_age <= 0.080f)
  {
    f error = input->gimbal_pitch - leg_roll_balance_target;
    f direction = (leg_roll_balance_direction >= 0.0f) ? 1.0f : -1.0f;
    f i_limit = LIMIT(fabsf(leg_roll_balance_i_limit), 0.0f, 8.0f);
    f ki = LIMIT(leg_roll_balance_ki, 0.0f, 3.0f);
    f pd_cmd = leg_roll_balance_kp * error +
               leg_roll_balance_kd * leg_roll_rate_filtered;
    f candidate = direction * (pd_cmd + leg_roll_balance_integral);
    f output_limit = LIMIT(fabsf(leg_roll_balance_limit), 0.0f, 40.0f);
    f selected_height = (candidate >= 0.0f) ?
        leg_normalized_height(input->left_motor->mang,
                              LEFT_LEG_MIN_MANG, LEFT_LEG_MAX_MANG) :
        leg_normalized_height(input->right_motor->mang,
                              RIGHT_LEG_MIN_MANG, RIGHT_LEG_MAX_MANG);
    f selected_retract_scale = LIMIT(selected_height /
                                         LEG_ROLL_RETRACT_SOFT_ZONE,
                                     0.0f, 1.0f);

    // 积分只消除静态气弹簧/重力差，不参与快速侧倾。接近输出饱和时冻结，
    // 进入目标死区时连续泄漏，避免静摩擦释放后积分突然翻转造成左右摆动。
    if (fabsf(error) > LEG_ROLL_INTEGRAL_ERROR_DEADZONE &&
        fabsf(leg_roll_rate_filtered) < LEG_ROLL_INTEGRAL_RATE_GATE &&
        fabsf(candidate) < leg_max(0.0f, output_limit - 0.5f) &&
        selected_retract_scale > 0.20f)
    {
      leg_roll_balance_integral += ki * error * dt;
    }
    else if (fabsf(error) <= LEG_ROLL_INTEGRAL_ERROR_DEADZONE)
    {
      leg_roll_balance_integral = leg_decay_to_zero(
          leg_roll_balance_integral, dt, LEG_ROLL_INTEGRAL_LEAK_TIME);
    }
    leg_roll_balance_integral = LIMIT(leg_roll_balance_integral,
                                      -i_limit, i_limit);
    leg_roll_balance_i_output = direction * leg_roll_balance_integral;

    target_cmd = direction * (pd_cmd + leg_roll_balance_integral);
    target_cmd = LIMIT(target_cmd, -output_limit, output_limit);
  }
  else
  {
    leg_roll_balance_integral = leg_decay_to_zero(
        leg_roll_balance_integral, dt, 0.20f);
    leg_roll_balance_i_output = 0.0f;
  }

  f cmd_alpha = dt / (LEG_ROLL_BALANCE_FILTER_TIME_CONSTANT + dt);
  leg_roll_balance_filtered += cmd_alpha *
                               (target_cmd - leg_roll_balance_filtered);
  return leg_roll_balance_filtered;
}

f leg_retract_soft_scale(f angle, f min_angle, f max_angle)
{
  f height = leg_normalized_height(angle, min_angle, max_angle);
  return LIMIT(height / LEG_ROLL_RETRACT_SOFT_ZONE, 0.0f, 1.0f);
}

void leg_apply_roll_retract_only(const LegControlInput *input,
                                        f roll_balance_cmd,
                                        f torque_limit)
{
  if (roll_balance_cmd > 0.0f)
  {
    // Positive physical roll: retract the left leg only.
    f scale = leg_retract_soft_scale(input->left_motor->mang,
                                     LEFT_LEG_MIN_MANG, LEFT_LEG_MAX_MANG);
    f requested = roll_balance_cmd * scale;
    f left_before = leg_output.left_torque;
    leg_output.left_torque = LIMIT(left_before - requested,
                                   -torque_limit, torque_limit);

    // If the selected leg is already at its retract limit, release part of
    // the other leg's retract torque. This preserves roll authority without
    // commanding the opposite leg to extend.
    f remaining = requested - (left_before - leg_output.left_torque);
    if (remaining > 0.0f && leg_output.right_torque > 0.0f)
      leg_output.right_torque = leg_max(0.0f,
                                        leg_output.right_torque - remaining);
  }
  else if (roll_balance_cmd < 0.0f)
  {
    // Negative physical roll: retract the right mirrored leg only.
    f scale = leg_retract_soft_scale(input->right_motor->mang,
                                     RIGHT_LEG_MIN_MANG, RIGHT_LEG_MAX_MANG);
    f requested = -roll_balance_cmd * scale;
    f right_before = leg_output.right_torque;
    leg_output.right_torque = LIMIT(right_before + requested,
                                    -torque_limit, torque_limit);

    f remaining = requested - (leg_output.right_torque - right_before);
    if (remaining > 0.0f && leg_output.left_torque < 0.0f)
      leg_output.left_torque = leg_min(0.0f,
                                       leg_output.left_torque + remaining);
  }
}

f leg_height_hold_update(const LegControlInput *input, f dt)
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

void leg_sync_output(void)
{
  leg_output.mit_kd = leg_dm_mit_kd;
  leg_output.roll_cmd = leg_roll_cmd;
  leg_output.pitch_cmd = leg_pitch_cmd;
  leg_output.dynamic_state = (uint8_t)leg_dynamic_state;
}


