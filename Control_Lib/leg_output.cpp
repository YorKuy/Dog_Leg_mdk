#include "leg_internal.h"

f leg_limit_retract_torque(f torque, f angle_range, f limit)
{
  if (torque * angle_range >= 0.0f)
    return torque;
  return LIMIT(torque, -limit, limit);
}

f leg_unload_slew(f target, f last, f angle_range, f dt)
{
  f target_motion = target * angle_range;
  f last_motion = last * angle_range;
  f step = LIMIT(leg_normal_drive_slew_rate, 100.0f, 5000.0f) * dt;

  if (target_motion < last_motion && target_motion < 0.0f)
    step = LEG_UNLOAD_RETRACT_SLEW_RATE * dt;
  else if (target_motion > last_motion && last_motion < 0.0f)
    step = LEG_UNLOAD_BRAKE_SLEW_RATE * dt;

  return last + LIMIT(target - last, -step, step);
}

// 普通状态继续驱动时保留原 1250 Nm/s 响应；当力矩变化方向正在阻止
// 关节现有运动时允许更快变化，减少滤波和统一斜率造成的制动相位滞后。
f leg_normal_slew(f target, f last, f motor_speed, f dt)
{
  f delta = target - last;
  f drive_rate = LIMIT(leg_normal_drive_slew_rate, 100.0f, 5000.0f);
  f brake_rate = LIMIT(leg_normal_brake_slew_rate, drive_rate, 8000.0f);
  f rate = drive_rate;
  if (fabsf(motor_speed) > 0.05f && delta * motor_speed < 0.0f)
    rate = brake_rate;
  return last + LIMIT(delta, -rate * dt, rate * dt);
}

// 对最终单电机力矩进行在线S曲线规划。14Nm是从当前已整形命令向原始
// 平衡目标观察的近端窗口，不是一次跳变14Nm。快速收腿仍属于同一平衡
// 输出；这里只根据力矩变化是否正在阻止关节现有运动选择驱动/制动带宽。
f leg_normal_scurve(f target, f last, f motor_speed, f dt,
                    f *rate_state, f *stage_target, u8 emergency_bypass)
{
  if (rate_state == 0)
    return leg_normal_slew(target, last, motor_speed, dt);

  f delta = target - last;
  f window = LIMIT(fabsf(leg_torque_stage_window), 2.0f, 40.0f);
  f stage = last + LIMIT(delta, -window, window);
  if (stage_target != 0)
    *stage_target = stage;

  // 姿态紧急条件不能再叠加S曲线相位；保留原来已经验证的驱动/制动限斜率。
  if (!leg_enable_torque_scurve || emergency_bypass)
  {
    *rate_state = 0.0f;
    if (stage_target != 0)
      *stage_target = target;
    return leg_normal_slew(target, last, motor_speed, dt);
  }

  if (fabsf(delta) < 0.0001f)
  {
    *rate_state = 0.0f;
    if (stage_target != 0)
      *stage_target = target;
    return target;
  }

  f drive_rate = LIMIT(leg_normal_drive_slew_rate, 100.0f, 5000.0f);
  f brake_rate = LIMIT(leg_normal_brake_slew_rate, drive_rate, 8000.0f);
  u8 braking = ((fabsf(motor_speed) > 0.05f && delta * motor_speed < 0.0f) ||
                delta * (*rate_state) < 0.0f);
  f rate_limit = braking ? brake_rate : drive_rate;
  f jerk = braking ? leg_torque_brake_jerk : leg_torque_drive_jerk;
  jerk = LIMIT(fabsf(jerk), 10000.0f, 5000000.0f);

  // sqrt(2*jerk*distance)使输出接近最终目标时提前降低力矩斜率，避免越过
  // 目标。目标较远时14Nm窗口只限制规划视距，不会形成停顿或阶梯输出。
  f distance = fabsf(stage - last);
  f stop_rate = sqrtf(2.0f * jerk * distance);
  f desired_rate = leg_min(rate_limit, stop_rate);
  if (delta < 0.0f)
    desired_rate = -desired_rate;

  f rate_step = jerk * dt;
  *rate_state += LIMIT(desired_rate - *rate_state, -rate_step, rate_step);
  // 制动结束转回普通驱动时，允许之前较高的制动力矩斜率按jerk连续下降，
  // 禁止直接从2500Nm/s截断到1250Nm/s而重新制造斜率突变。
  *rate_state = LIMIT(*rate_state, -brake_rate, brake_rate);

  f next = last + *rate_state * dt;
  if ((delta > 0.0f && next >= target) ||
      (delta < 0.0f && next <= target))
  {
    next = target;
    *rate_state = 0.0f;
  }
  return next;
}
// 加入关节速度阻尼并限制最终力矩。
f leg_apply_joint_damping_custom(f torque_cmd,
                                        MOTOR_DM *motor,
                                        f torque_limit,
                                        f damping_gain,
                                        f speed_deadzone,
                                        f angle_range)
{
  f motor_sp = (motor != 0) ? motor->sp : 0;
  f abs_speed = fabsf(motor_sp);
  if (abs_speed <= speed_deadzone)
    motor_sp = 0.0f;
  else
    motor_sp = (motor_sp > 0.0f) ?
                   (abs_speed - speed_deadzone) :
                  -(abs_speed - speed_deadzone);

  // 左右电机镜像，motor_sp 正负不能直接代表伸/收腿；motor_sp 与机械角
  // 范围同号表示归一化腿长正在增加（伸腿），反号表示收腿。
  f physical_height_speed_sign = motor_sp * angle_range;
  f direction_scale = (physical_height_speed_sign >= 0.0f) ?
      LIMIT(leg_joint_damping_extend_scale, 0.5f, 2.0f) :
      LIMIT(leg_joint_damping_retract_scale, 0.5f, 2.0f);
  return LIMIT(torque_cmd - damping_gain * direction_scale * motor_sp,
               -torque_limit, torque_limit);
}

f leg_remaining_output(f torque_limit, f used_output)
{
  f remaining = torque_limit - fabsf(used_output);
  return (remaining > 0.0f) ? remaining : 0.0f;
}
// 镜像安装混控：roll 左右相反，pitch 左右相同。
void leg_set_balance_output_custom(const LegControlInput *input,
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
                                     speed_deadzone,
                                     LEFT_LEG_MAX_MANG - LEFT_LEG_MIN_MANG);

  leg_output.right_torque =
      leg_apply_joint_damping_custom(right_leg_cmd,
                                     input->right_motor,
                                     torque_limit,
                                     damping_gain,
                                     speed_deadzone,
                                     RIGHT_LEG_MAX_MANG - RIGHT_LEG_MIN_MANG);
}

// Player 手动腿长模式的附加平衡：不覆盖腿长串级输出，只叠加
// 物理 pitch 的对称力矩。实际 roll 仍由调用方走单侧收腿路径。
void leg_apply_balance_overlay(f pitch_cmd, f torque_limit)
{
  f pitch_out = LIMIT(LEG_ROLL_BALANCE_SIGN * pitch_cmd,
                      -torque_limit, torque_limit);
  leg_output.left_torque = LIMIT(leg_output.left_torque + pitch_out,
                                 -torque_limit, torque_limit);
  leg_output.right_torque = LIMIT(leg_output.right_torque - pitch_out,
                                  -torque_limit, torque_limit);
}
