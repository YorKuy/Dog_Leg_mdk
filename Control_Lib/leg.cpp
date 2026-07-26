#include "leg.h"
#include "SMC.h"
#include "main.h"

#include <math.h>

typedef float f;
typedef uint8_t u8;
//当前：动态单数据补偿，smc快速响应，pid缓力矩校准补偿，离地缓冲，加速度腿速前馈
//后续发展：lqr状态矩阵建模，vmc虚拟力矩转化关节力矩，MPC模型扰动与前馈

#define LEFT_LEG_MAX_MANG -0.59207F
#define LEFT_LEG_MIN_MANG -1.34095F 
#define RIGHT_LEG_MAX_MANG -2.59529F 
#define RIGHT_LEG_MIN_MANG -1.92694F  

// 电机阻尼与输出保护。KD 和软件阻尼都增大时抑振更强，但腿会变钝且发热增加。
#define LEG_DM_MIT_KD 3.5f                       // DM MIT 内部速度阻尼；建议每次只加/减 0.2，范围不要超过驱动器 KD_MAX=5
#define LEG_JOINT_DAMPING_GAIN 0.60f             // 软件关节速度阻尼 Nm/(rad/s)；增大可抑制连杆振荡，过大会阻碍伸缩
#define LEG_JOINT_DAMPING_SPEED_DEADZONE 0.20f   // 速度小于该值时不加软件阻尼；减小更灵敏但更容易放大速度噪声

#define LEG_PITCH_TARGET_ANGLE 0.0f      // 物理 roll 默认水平目标，degree；当前用于初始化 leg_roll_balance_target
#define LEG_PITCH_BALANCE_SIGN -1.0f     // 预留的物理 roll 对称混控符号；当前 leg_pitch_cmd=0，通常不要修改
#define LEG_PITCH_OUTPUT_LIMIT 18.0f     // 预留物理 roll 对称混控限幅，Nm；当前主路径基本不生效
#define LEG_ROLL_BALANCE_SIGN -1.0f      // 物理 pitch SMC 到镜像电机的符号；已实车验证，禁止作为增益调节
#define LEG_ROLL_KEEP_TARGET_ANGLE 1.0f  // 物理 pitch 平衡目标，degree；车体静止前后水平时按 IMU 平均值校准
#define LEG_SMC_OUTPUT_SLEW_STEP 1.25f   // 最终单电机每 1 ms 最大力矩变化，Nm；增大响应快但冲击大，减小平滑但滞后

// 动态补偿参数，主控制周期为 1 kHz。时间常数增大更平滑但响应更慢，减小则相反。
#define LEG_CONTROL_DT_DEFAULT 0.001f                 // 输入 dt 异常时采用的默认周期，s；必须与 1 kHz 控制周期一致
#define LEG_ACCEL_FF_ACCEL_TIME_CONSTANT 0.016f       // 加速前馈一阶滤波时间，s；振荡时可增大，补偿迟钝时可减小
#define LEG_ACCEL_FF_BRAKE_TIME_CONSTANT 0.006f       // 制动前馈滤波时间，s；减小可更快应对急停，但过小会产生冲击
#define LEG_ACCEL_FF_DECAY_TIME 0.006f                // 加速度结束后前馈衰减时间，s；增大会残留更久，可能导致急停后反弹
#define LEG_DYNAMIC_AUX_LIMIT 8.0f                    // 加速度前馈+腿速阻尼+高度保持的总限幅，Nm；增大辅助更强但会抢占 pitch 输出
#define LEG_HEIGHT_DAMPING_LIMIT 6.0f                 // 腿高速度阻尼最大输出，Nm；高速坡振荡时可小幅增加
#define LEG_HEIGHT_SPEED_FILTER_TIME_CONSTANT 0.016f  // 归一化腿高速度滤波时间，s；增大可降噪但阻尼相位更滞后
#define LEG_HEIGHT_DAMPING_TO_ROLL_SIGN 1.0f

// 物理 roll 单侧收腿补偿。正命令收左腿，负命令收右腿；这里只调响应速度和限位。
#define LEG_ROLL_BALANCE_FILTER_TIME_CONSTANT 0.008f  // 物理 roll 力矩滤波时间，s；减小响应快，增大可减轻左右抖动
#define LEG_ROLL_RATE_FILTER_TIME_CONSTANT 0.020f     // 物理 roll 角速度滤波时间，s；角速度噪声大时增大，单边桥响应慢时减小
#define LEG_ROLL_RETRACT_SOFT_ZONE 0.05f              // 接近完全收腿端的软保护比例；增大可更早减力，但会减少限位附近 roll 权限

// 自动坡面支撑识别与单向高度保持。leg_enable_slope_hold=0 时以下识别参数不生效。
#define LEG_SLOPE_FORWARD_ENTER 120.0f       // 允许进入坡面候选的最小前进命令；减小更容易触发，也更容易平地误触发
#define LEG_SLOPE_FORWARD_EXIT 60.0f         // 低于该前进命令退出支撑；增大退出更早，减小可保持更久
#define LEG_SLOPE_PITCH_ENTER 2.0f           // 物理 pitch 偏差触发阈值，degree；减小更灵敏但容易被颠簸触发
#define LEG_SLOPE_STALL_ENTER 0.20f          // 四轮平均失速率阈值，0~1；减小更容易识别坡面/障碍
#define LEG_SLOPE_SMC_ENTER 6.0f             // 物理 pitch SMC 力矩阈值，Nm；减小更容易触发支撑
#define LEG_SLOPE_CONFIRM_TIME 0.032f         // 条件持续确认时间，s；增大可抗误触发但识别变慢
#define LEG_SLOPE_CANDIDATE_TIMEOUT 0.300f    // 候选状态最长等待时间，s；太短可能来不及确认，太长可能卡在候选态
#define LEG_SUPPORT_TIMEOUT 2.500f            // 支撑保持最长时间，s；增大允许长坡保持，但错误状态持续更久
#define LEG_HEIGHT_HOLD_DEADZONE 0.005f       // 归一化腿高保持死区；增大可减抖但高度波动增大
#define LEG_HEIGHT_REF_RELEASE_RATE 0.01f     // 高度参考向收腿方向的释放速度，归一化高度/s；增大释放快但保持变弱
#define LEG_HEIGHT_TO_ROLL_SIGN -1.0f         // 高度保持加到物理 pitch 轴的符号；机构方向参数，不应作为增益调整
#define LEG_HEIGHT_EMERGENCY_PITCH 8.0f       // 超过该物理 pitch 偏差时撤销高度保持，degree；减小更保守
#define LEG_HEIGHT_EMERGENCY_RATE 60.0f       // 超过该物理 pitch 角速度时撤销高度保持，degree/s；减小更早让权给 SMC

// 后轮卸载/离地缓冲。leg_enable_unload_catch=0 时以下缓冲参数不生效。
#define LEG_UNLOAD_HEIGHT_SPEED -0.25f          // 判定快速收腿的归一化腿高速度；绝对值减小会更容易触发
#define LEG_UNLOAD_TORQUE_DROP_RATIO 0.70f      // 当前反馈力矩低于峰值的该比例视为卸载；增大更容易触发
#define LEG_UNLOAD_PITCH_RATE 15.0f             // 可替代力矩下降条件的物理 pitch 角速度阈值，degree/s；减小更敏感
#define LEG_UNLOAD_CONFIRM_TIME 0.008f           // 卸载条件确认时间，s；增大可抗噪但缓冲介入更晚
#define LEG_UNLOAD_MIN_TIME 0.080f               // 卸载缓冲最短保持时间，s；增大保护更充分但动作更慢
#define LEG_UNLOAD_MAX_TIME 0.200f               // 卸载缓冲最长时间，s；增大可延长保护但影响后续收敛
#define LEG_UNLOAD_TORQUE_LIMIT 30.0f            // 卸载状态最大主动收腿力矩，Nm；减小冲击更小，但可能收腿不足
#define LEG_UNLOAD_RETRACT_SLEW_PER_STEP 0.30f   // 卸载时收腿力矩每 1 ms 最大增量，Nm，即 300 Nm/s；减小更柔和
#define LEG_UNLOAD_BRAKE_SLEW_PER_STEP 0.80f     // 卸载时制动力矩每 1 ms 最大变化，Nm，即 800 Nm/s；增大制动更及时
#define LEG_UNLOAD_DAMPING_GAIN 1.80f            // 卸载时软件关节速度阻尼；增大抑制离地后甩腿，过大会阻碍动作
#define LEG_UNLOAD_MIT_KD 4.20f                  // 卸载时 DM MIT KD；增大电机阻尼更强，禁止超过驱动器 KD_MAX=5
#define LEG_SETTLE_TIME 0.250f                   // 卸载后柔和恢复持续时间，s；增大更稳但回到正常控制更慢

// FAST/PLAYER 手动腿长串级 PID。实际 PID 增益在下方 PID_class 构造函数中，
// LEG_FAST_HEIGHT_KP/KD/UP_FF/STEP_DEADZONE 是旧版预留宏，当前没有进入计算。
#define LEG_FAST_HEIGHT_KP 242.0f                  // 旧版预留腿高 Kp，当前未使用，修改无效果
#define LEG_FAST_HEIGHT_KD 0.60f                   // 旧版预留腿高 Kd，当前未使用，修改无效果
#define LEG_FAST_HEIGHT_LIMIT 38.0f                // 角度 PID 接近该输出比例时累加前馈；增大后前馈介入更晚
#define LEG_FAST_HEIGHT_UP_FF 5.0f                 // 旧版预留上抬前馈，当前未使用，修改无效果
#define LEG_FAST_HEIGHT_STEP_DEADZONE 0.00005f     // 旧版预留目标步进死区，当前未使用，修改无效果
#define LEG_FAST_HEIGHT_TARGET_STEP_DIV 6600.0f    // 手动腿长目标步进除数；增大腿长变化更慢，减小更快
#define LEG_FAST_HEIGHT_FF_ADD 0.10f               // 误差持续增大时每周期累加的前馈；增大顶腿更强但容易冲击
#define LEG_FAST_HEIGHT_FF_DECAY 0.96f             // 前馈无须继续增加时的保留比例；越接近 1 衰减越慢
#define LEG_FAST_HEIGHT_FF_LIMIT 50.0f             // 腿长误差增长前馈限幅，Nm；增大可能超过最终输出限幅而无额外效果
#define LEG_FAST_HEIGHT_OUT_LIMIT 40.0f            // 手动腿长最终单电机力矩限幅，Nm；增大提升能力也增加结构负担
#define LEG_FAST_HEIGHT_ERROR_DEADZONE 0.010f       // 腿长角度误差死区，rad；增大可减抖但保持精度下降
#define LEG_FAST_HEIGHT_ERROR_GROW_DEADZONE 0.0003f // 判断误差正在增大的阈值，rad；减小会更频繁累加前馈
#define LEG_FAST_HEIGHT_PID_FULL_RATIO 0.90f        // PID 输出达到限幅的该比例时提前累加前馈；减小会更早介入

static PID_class left_control_mang(120.0f, 0.0f, 0.0f, 84.0f, 0.0f, 0.0f, 84.0f, 0.06f, 0.0f),
    left_control_sp(0.32f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f, 20.0f, 0.20f, 0.0f),
    right_control_mang(120.0f, 0.0f, 0.0f, 84.0f, 0.0f, 0.0f, 84.0f, 0.06f, 0.0f),
    right_control_sp(0.32f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f, 20.0f, 0.20f, 0.0f);

// 物理 pitch SMC 参数顺序：C, K, C2, error_eps, u_max, J, epsilon。
// C/K/J 增大都会增强响应并提高振荡风险；C2 增大可减小长坡静差但增加卸载后过冲；
// error_eps 只决定积分泄漏范围；u_max 为原始 SMC 输出上限；epsilon 增大切换项和抖振。
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

// 以下 volatile 变量可在 Ozone 中在线修改。开关只允许写 0/1；调参时一次只改一个量。
volatile uint8_t leg_enable_accel_feedforward = 1;  // 加减速物理 pitch 前馈：1启用；排查前馈方向/振荡时可临时置0对比
volatile uint8_t leg_enable_forward_jerk_limit = 1; // 前进命令 S 曲线：1启用；关闭后加减速更直接，惯性冲击也更大
volatile uint8_t leg_enable_height_damping = 1;     // 归一化腿高速度阻尼：1启用；高速坡振荡时应保持开启
volatile uint8_t leg_enable_slope_hold = 0;         // 自动坡面识别和高度保持：当前默认关闭，完成识别验证后再开启
volatile uint8_t leg_enable_unload_catch = 0;       // 后轮离地收腿缓冲：当前默认关闭，先确认卸载检测没有误触发
volatile uint8_t leg_enable_roll_balance = 1;       // 物理 roll 单侧收腿平衡：1启用；保护架检查方向时可快速关闭

volatile float leg_accel_ff_accel_gain = 2.50f;     // 加速前馈增益；增大下压更强，过大会在加速结束后反弹
volatile float leg_accel_ff_brake_gain = 2.80f;     // 制动前馈增益；增大急停补偿更强，过大会反向冲击
volatile float leg_accel_ff_limit = 6.0f;           // 单独加速度前馈限幅，Nm；急停振荡先减到5/4.5，补偿不足再增加
volatile float leg_height_damping_gain = 4.5f;      // 腿高速度阻尼增益；坡面往复振荡时每次加0.5，腿变钝时回退

volatile float leg_height_hold_kp = 60.0f;          // 支撑高度误差刚度，Nm/归一化高度；增大保持更硬，也更容易上下振荡
volatile float leg_height_hold_kd = 3.0f;           // 支撑状态收腿速度阻尼；增大可抑制回落，过大会阻碍正常收腿
volatile float leg_height_hold_limit = 6.0f;        // 高度保持最大附加力矩，Nm；增大保持更强但更会抢占物理 pitch 控制

volatile float leg_roll_balance_kp = 2.10f;         // 物理 roll 角度增益，Nm/degree；持续侧倾时每次加0.2，左右摇摆时减小
volatile float leg_roll_balance_kd = 0.18f;         // 物理 roll 角速度阻尼，Nm/(degree/s)；来回摇摆时每次加0.03，噪声抖动时减小
volatile float leg_roll_balance_limit = 14.0f;      // 物理 roll 最大单侧收腿补偿，Nm；命令长期饱和才逐步增加到16
volatile float leg_roll_balance_direction = 1.0f;  // 物理 roll 方向，只允许1或-1；若倾斜后补偿使其更严重就翻转
volatile float leg_roll_balance_target = LEG_PITCH_TARGET_ANGLE; // 物理 roll 水平零点，degree；填写车体水平静止时 Gimbal_Roll 平均值

// 只读诊断计数，不是控制增益。正常运行 fail/consecutive_fail 应保持不增长。
volatile uint32_t leg_dm_pair_send_ok_count;          // 左右 DM 同周期成对发送成功累计次数
volatile uint32_t leg_dm_pair_send_fail_count;        // 因邮箱不足或发送失败导致整对未成功的累计次数
volatile uint16_t leg_dm_pair_send_consecutive_fail;  // 连续成对发送失败次数；持续增长表示 CAN 带宽或发送异常

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
                      (input->gimbal_pitch - leg_roll_balance_target) +
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
    leg_roll_smc.Reset();
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
    if (leg_smc_mode_last)
      leg_roll_smc.Reset();
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
    if (leg_smc_mode_last == 0)
      leg_roll_smc.Reset();
    leg_roll_smc.ref = LEG_ROLL_KEEP_TARGET_ANGLE;
    leg_roll_smc.SMC_Tick(LEG_ROLL_KEEP_TARGET_ANGLE, 0.0f, 0.0f,
                          input->gimbal_roll, input->gimbal_roll_acc);
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
    if (leg_smc_mode_last)
      leg_roll_smc.Reset();
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
