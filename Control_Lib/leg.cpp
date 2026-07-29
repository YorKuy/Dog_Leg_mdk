#include "leg_internal.h"

//当前：动态单数据补偿，smc快速响应，pid缓力矩校准补偿，离地缓冲，加速度腿速前馈
//后续发展：lqr状态矩阵建模，vmc虚拟力矩转化关节力矩，MPC模型扰动与前馈
static SMC_PITCH leg_roll_smc(48, 65, 1.05f, 1.5f, 8000, 0.8f, 1.0f);
static UpDown_check_class leg_mode_key(0);
static u8 leg_control_mode = 0;
f leg_dm_mit_kd = LEG_DM_MIT_KD;
f leg_roll_cmd;
f leg_pitch_cmd;
LegControlOutput leg_output = {0};

// 以下 volatile 变量可在 Ozone 中在线修改。开关只允许写 0/1；调参时一次只改一个量。
volatile uint8_t leg_enable_pitch_balance = 1;     // 物理 pitch 主控制总开关；置0仅用于保护架单独验证物理 roll 极性
volatile uint8_t leg_enable_accel_feedforward = 1;  // 加减速物理 pitch 前馈：1启用；排查前馈方向/振荡时可临时置0对比
volatile uint8_t leg_enable_forward_jerk_limit = 1; // 前进命令 S 曲线：1启用；关闭后加减速更直接，惯性冲击也更大
volatile uint8_t leg_enable_height_damping = 1;     // 归一化腿高速度阻尼：1启用；高速坡振荡时应保持开启
volatile uint8_t leg_enable_slope_hold = 0;         // 自动坡面识别和高度保持：当前默认关闭，完成识别验证后再开启
volatile uint8_t leg_enable_unload_catch = 0;       // 后轮离地收腿缓冲：当前默认关闭，先确认卸载检测没有误触发
volatile uint8_t leg_enable_roll_balance = 1;       // 物理 roll 单侧收腿平衡：1启用；保护架检查方向时可快速关闭
volatile uint8_t leg_enable_player_balance_overlay = 1; // Player手动腿长时叠加pitch/roll平衡；关闭后恢复纯腿长串级旧行为
volatile uint8_t leg_enable_torque_scurve = 1;      // 最终平衡力矩S曲线：1启用；CONTROL/PLAYER共用，置0立即恢复原线性限斜率

volatile float leg_accel_ff_accel_gain = 2.50f;     // 加速前馈增益；增大下压更强，过大会在加速结束后反弹
volatile float leg_accel_ff_brake_gain = 2.80f;     // 制动前馈增益；增大急停补偿更强，过大会反向冲击
volatile float leg_accel_ff_limit = 6.0f;           // 单独加速度前馈限幅，Nm；急停振荡先减到5/4.5，补偿不足再增加
volatile float leg_height_damping_gain = 5.0f;      // 腿高速度阻尼增益；坡面往复振荡时每次加0.5，腿变钝时回退

volatile float leg_height_hold_kp = 60.0f;          // 支撑高度误差刚度，Nm/归一化高度；增大保持更硬，也更容易上下振荡
volatile float leg_height_hold_kd = 3.0f;           // 支撑状态收腿速度阻尼；增大可抑制回落，过大会阻碍正常收腿
volatile float leg_height_hold_limit = 6.0f;        // 高度保持最大附加力矩，Nm；增大保持更强但更会抢占物理 pitch 控制

volatile float leg_roll_balance_kp = 3.10f;         // 物理 roll 角度增益，Nm/degree；持续侧倾时每次加0.2，左右摇摆时减小
volatile float leg_roll_balance_kd = 0.28f;         // 物理 roll 角速度阻尼，Nm/(degree/s)；来回摇摆时每次加0.03，噪声抖动时减小
volatile float leg_roll_balance_ki = 0.50f;         // 物理 roll 静差积分，Nm/(degree*s)；只处理慢性偏差，振荡时先减小而不是继续加 Kp
volatile float leg_roll_balance_i_limit = 4.0f;     // 物理 roll 积分输出限幅，Nm；气弹簧负载很大也不要一次超过 4
volatile float leg_roll_balance_limit = 18.0f;      // 物理 roll 最大单侧收腿补偿，Nm；命令长期饱和先查软限位，禁止继续盲目增大
volatile float leg_roll_balance_direction = -1.0f;  // 物理 roll 方向，只允许1或-1；若倾斜后补偿使其更严重就翻转
volatile float leg_roll_balance_target = LEG_PITCH_TARGET_ANGLE; // 物理 roll 水平零点，degree；填写车体水平静止时 Gimbal_Roll 平均值
volatile float leg_roll_balance_i_output;           // 只读：乘过方向后的 roll 积分力矩，Nm；长期到限幅表示零点或机械权限不足

volatile float leg_joint_damping_gain = 0.60f;      // 普通模式软件关节阻尼，Nm/(rad/s)；气弹簧助伸导致振荡时每次加0.1
volatile float leg_joint_damping_speed_deadzone = 0.05f; // 连续速度死区，rad/s；原0.20会造成低速阻尼突然消失
volatile float leg_joint_damping_extend_scale = 1.25f; // 伸腿阻尼倍率；气弹簧会助伸，倍率应略大于1以防突然弹起
volatile float leg_joint_damping_retract_scale = 0.75f; // 收腿阻尼倍率；气弹簧本身抗压缩，倍率过大会使收腿迟钝并积累控制误差
volatile float leg_height_speed_filter_time = 0.012f; // 腿高速度滤波时间，s；原26ms在快速坡面上阻尼介入偏迟
volatile float leg_normal_drive_slew_rate = 1250.0f; // 普通继续驱动力矩变化率，Nm/s；保持原1.25Nm/ms响应
volatile float leg_normal_brake_slew_rate = 2500.0f; // 普通制动/卸力变化率，Nm/s；加快刹住已发生的腿部运动
volatile float leg_torque_stage_window = 14.0f;      // 力矩规划近端窗口，Nm；减小更柔和，过小会放大平衡相位滞后
volatile float leg_torque_drive_jerk = 250000.0f;    // 继续驱动时力矩斜率建立速度，Nm/s^2；当前约5ms达到1250Nm/s
volatile float leg_torque_brake_jerk = 1250000.0f;   // 阻止关节现有运动时的jerk，Nm/s^2；当前约2ms达到2500Nm/s

// 只读诊断计数，不是控制增益。正常运行 fail/consecutive_fail 应保持不增长。
volatile uint32_t leg_dm_pair_send_ok_count;          // 左右 DM 同周期成对发送成功累计次数
volatile uint32_t leg_dm_pair_send_fail_count;        // 因邮箱不足或发送失败导致整对未成功的累计次数
volatile uint16_t leg_dm_pair_send_consecutive_fail;  // 连续成对发送失败次数；持续增长表示 CAN 带宽或发送异常

LegDynamicState leg_dynamic_state = LEG_DYNAMIC_NORMAL;
f leg_dynamic_state_time;
f leg_slope_confirm_time;
f leg_unload_confirm_time;
f leg_height_reference;
f leg_height_last;
f leg_height_speed_filtered;
f leg_feedback_torque_peak;
f leg_accel_ff_filtered;
f leg_last_forward_cmd;
f leg_forward_motion_sign;
f leg_roll_rate_filtered;
f leg_roll_balance_filtered;
f leg_roll_balance_integral;
u8 leg_height_initialized;
u8 leg_forward_initialized;

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
  static f leg_height_last_left_torque, leg_height_last_right_torque;
  static f leg_smc_left_torque_rate, leg_smc_right_torque_rate;
  static f leg_height_left_torque_rate, leg_height_right_torque_rate;
  static u8 leg_height_mode_last = 0;
  static u8 leg_smc_mode_last = 0;
  static u8 leg_pitch_balance_last = 0;
  if (input == 0 || input->left_motor == 0 || input->right_motor == 0)
  {
    leg_output.left_torque = 0;
    leg_output.right_torque = 0;
    leg_output.left_raw_torque = 0;
    leg_output.right_raw_torque = 0;
    leg_output.left_stage_torque = 0;
    leg_output.right_stage_torque = 0;
    leg_dm_mit_kd = LEG_DM_MIT_KD;
    leg_roll_cmd = 0;
    leg_pitch_cmd = 0;
    leg_height_mode_last = 0;
    leg_smc_mode_last = 0;
    leg_pitch_balance_last = 0;
    leg_height_initialized = 0;
    leg_forward_initialized = 0;
    leg_roll_rate_filtered = 0.0f;
    leg_roll_balance_filtered = 0.0f;
    leg_roll_balance_integral = 0.0f;
    leg_roll_balance_i_output = 0.0f;
    leg_height_last_left_torque = 0.0f;
    leg_height_last_right_torque = 0.0f;
    leg_smc_left_torque_rate = 0.0f;
    leg_smc_right_torque_rate = 0.0f;
    leg_height_left_torque_rate = 0.0f;
    leg_height_right_torque_rate = 0.0f;
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
  leg_output.left_raw_torque = 0.0f;
  leg_output.right_raw_torque = 0.0f;
  leg_output.left_stage_torque = 0.0f;
  leg_output.right_stage_torque = 0.0f;

  // gimbal_roll是装配交换后的车体物理pitch。姿态越界时只旁路新增的
  // jerk限制，仍保留原有最终力矩限斜率，避免平衡响应被额外延迟。
  u8 torque_scurve_emergency =
      (fabsf(input->gimbal_roll - LEG_ROLL_KEEP_TARGET_ANGLE) >
           LEG_HEIGHT_EMERGENCY_PITCH ||
       fabsf(input->gimbal_roll_acc) > LEG_HEIGHT_EMERGENCY_RATE);

  u8 leg_height_mode_now = (input->yk_mode == FAST_CHASSIC ||
                            (input->yk_mode == PLAYER_MODE && leg_control_mode));

  if (leg_height_mode_now)
  {
    u8 player_height_overlay =
        (input->yk_mode == PLAYER_MODE && leg_control_mode &&
         leg_enable_player_balance_overlay);
    f player_pitch_overlay_cmd = 0.0f;

    leg_smc_mode_last = 0;
    if (!player_height_overlay)
    {
      if (leg_pitch_balance_last)
        leg_roll_smc.Reset();
      leg_pitch_balance_last = 0;
      leg_enter_dynamic_state(LEG_DYNAMIC_NORMAL);
      leg_accel_ff_filtered = 0.0f;
      leg_forward_initialized = 0;
      leg_roll_rate_filtered = 0.0f;
      leg_roll_balance_filtered = 0.0f;
      leg_roll_balance_integral = 0.0f;
      leg_roll_balance_i_output = 0.0f;
      leg_output.roll_balance_cmd = 0.0f;
    }
    else
    {
      leg_output.roll_balance_cmd = leg_roll_balance_update(input, dt);
      if (leg_enable_pitch_balance)
      {
        if (leg_pitch_balance_last == 0)
          leg_roll_smc.Reset();
        leg_pitch_balance_last = 1;
        leg_roll_smc.ref = LEG_ROLL_KEEP_TARGET_ANGLE;
        leg_roll_smc.SMC_Tick(LEG_ROLL_KEEP_TARGET_ANGLE, 0.0f, 0.0f,
                              input->gimbal_roll, input->gimbal_roll_acc);
        leg_output.smc_cmd = torque_return(leg_roll_smc.u);
        leg_update_dynamic_state(input, leg_output.smc_cmd, dt);
        leg_output.accel_ff_cmd = leg_accel_feedforward_update(input, dt);
      }
      else
      {
        if (leg_pitch_balance_last)
          leg_roll_smc.Reset();
        leg_pitch_balance_last = 0;
        leg_enter_dynamic_state(LEG_DYNAMIC_NORMAL);
        leg_output.smc_cmd = 0.0f;
        leg_output.accel_ff_cmd = 0.0f;
        leg_accel_ff_filtered = 0.0f;
        leg_forward_initialized = 0;
      }
      // 高度串级已经直接控制腿长，避免再叠加同一腿速阻尼。
      leg_output.height_damping_cmd = 0.0f;
      leg_output.height_hold_cmd = 0.0f;
      player_pitch_overlay_cmd = leg_output.smc_cmd +
                                 leg_output.accel_ff_cmd;
    }
    if (leg_height_mode_last == 0)
    {
      leg_init_height_target(input, &now_left_mang_cmd, &now_right_mang_cmd);
      leg_height_last_left_torque = leg_output.left_torque;
      leg_height_last_right_torque = leg_output.right_torque;
      leg_height_left_torque_rate = 0.0f;
      leg_height_right_torque_rate = 0.0f;
    }
    leg_height_mode_last = 1;
    leg_height_control_update(input, &now_left_mang_cmd, &now_right_mang_cmd, dt);
    if (player_height_overlay)
    {
      leg_roll_cmd = player_pitch_overlay_cmd;
      leg_apply_balance_overlay(player_pitch_overlay_cmd, 40.0f);
      leg_apply_roll_retract_only(input, leg_output.roll_balance_cmd, 40.0f);
    }
    leg_output.left_raw_torque = leg_output.left_torque;
    leg_output.right_raw_torque = leg_output.right_torque;
    leg_height_last_left_torque = leg_normal_scurve(
        leg_output.left_raw_torque, leg_height_last_left_torque,
        input->left_motor->sp, dt, &leg_height_left_torque_rate,
        &leg_output.left_stage_torque, torque_scurve_emergency);
    leg_height_last_right_torque = leg_normal_scurve(
        leg_output.right_raw_torque, leg_height_last_right_torque,
        input->right_motor->sp, dt, &leg_height_right_torque_rate,
        &leg_output.right_stage_torque, torque_scurve_emergency);
    leg_output.left_torque = leg_height_last_left_torque;
    leg_output.right_torque = leg_height_last_right_torque;
  }
  else if (input->yk_mode == CONTROL_MODE || input->yk_mode == PLAYER_MODE ||
           input->yk_mode == XTL_MODE)
  {
    leg_height_mode_last = 0;
    f last_left_torque = leg_output.left_torque;
    f last_right_torque = leg_output.right_torque;
    leg_output.roll_balance_cmd = leg_roll_balance_update(input, dt);

    if (leg_enable_pitch_balance)
    {
      if (leg_pitch_balance_last == 0)
        leg_roll_smc.Reset();
      leg_pitch_balance_last = 1;
      leg_roll_smc.ref = LEG_ROLL_KEEP_TARGET_ANGLE;
      leg_roll_smc.SMC_Tick(LEG_ROLL_KEEP_TARGET_ANGLE, 0.0f, 0.0f,
                            input->gimbal_roll, input->gimbal_roll_acc);
      leg_output.smc_cmd = torque_return(leg_roll_smc.u);
      leg_update_dynamic_state(input, leg_output.smc_cmd, dt);
      leg_output.accel_ff_cmd = leg_accel_feedforward_update(input, dt);
      leg_output.height_damping_cmd = leg_height_damping_update();
      leg_output.height_hold_cmd = leg_height_hold_update(input, dt);

      f auxiliary_cmd = LIMIT(leg_output.accel_ff_cmd +
                                  leg_output.height_damping_cmd +
                                  leg_output.height_hold_cmd,
                              -LEG_DYNAMIC_AUX_LIMIT, LEG_DYNAMIC_AUX_LIMIT);
      leg_roll_cmd = leg_output.smc_cmd + auxiliary_cmd;
    }
    else
    {
      if (leg_pitch_balance_last)
        leg_roll_smc.Reset();
      leg_pitch_balance_last = 0;
      leg_enter_dynamic_state(LEG_DYNAMIC_NORMAL);
      leg_accel_ff_filtered = 0.0f;
      leg_forward_initialized = 0;
      leg_output.smc_cmd = 0.0f;
      leg_output.accel_ff_cmd = 0.0f;
      leg_output.height_damping_cmd = 0.0f;
      leg_output.height_hold_cmd = 0.0f;
      leg_roll_cmd = 0.0f;
    }
    leg_pitch_cmd = 0.0f;

    f damping_gain = LIMIT(leg_joint_damping_gain, 0.0f, 3.0f);
    f damping_deadzone = LIMIT(leg_joint_damping_speed_deadzone,
                               0.0f, 0.50f);
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
                                  damping_deadzone);
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
      leg_smc_left_torque_rate = 0.0f;
      leg_smc_right_torque_rate = 0.0f;
    }
    leg_output.left_raw_torque = leg_output.left_torque;
    leg_output.right_raw_torque = leg_output.right_torque;
    if (leg_enable_unload_catch &&
        leg_dynamic_state == LEG_DYNAMIC_UNLOAD_CATCH)
    {
      leg_smc_left_torque_rate = 0.0f;
      leg_smc_right_torque_rate = 0.0f;
      leg_output.left_stage_torque = leg_output.left_raw_torque;
      leg_output.right_stage_torque = leg_output.right_raw_torque;
      leg_smc_last_left_torque = leg_unload_slew(
          leg_output.left_raw_torque, leg_smc_last_left_torque,
          LEFT_LEG_MAX_MANG - LEFT_LEG_MIN_MANG, dt);
      leg_smc_last_right_torque = leg_unload_slew(
          leg_output.right_raw_torque, leg_smc_last_right_torque,
          RIGHT_LEG_MAX_MANG - RIGHT_LEG_MIN_MANG, dt);
    }
    else
    {
      leg_smc_last_left_torque = leg_normal_scurve(
          leg_output.left_raw_torque, leg_smc_last_left_torque,
          input->left_motor->sp, dt, &leg_smc_left_torque_rate,
          &leg_output.left_stage_torque, torque_scurve_emergency);
      leg_smc_last_right_torque = leg_normal_scurve(
          leg_output.right_raw_torque, leg_smc_last_right_torque,
          input->right_motor->sp, dt, &leg_smc_right_torque_rate,
          &leg_output.right_stage_torque, torque_scurve_emergency);
    }
    leg_output.left_torque = leg_smc_last_left_torque;
    leg_output.right_torque = leg_smc_last_right_torque;
    leg_smc_mode_last = 1;
  }
  else
  {
    leg_height_mode_last = 0;
    if (leg_pitch_balance_last)
      leg_roll_smc.Reset();
    leg_pitch_balance_last = 0;
    leg_smc_mode_last = 0;
    leg_enter_dynamic_state(LEG_DYNAMIC_NORMAL);
    leg_accel_ff_filtered = 0.0f;
    leg_forward_initialized = 0;
    leg_roll_rate_filtered = 0.0f;
    leg_roll_balance_filtered = 0.0f;
    leg_roll_balance_integral = 0.0f;
    leg_roll_balance_i_output = 0.0f;
    leg_roll_cmd = 0;
    leg_pitch_cmd = 0;
    leg_output.left_torque = 0;
    leg_output.right_torque = 0;
    leg_smc_left_torque_rate = 0.0f;
    leg_smc_right_torque_rate = 0.0f;
    leg_height_left_torque_rate = 0.0f;
    leg_height_right_torque_rate = 0.0f;
  }
  leg_sync_output();
}
