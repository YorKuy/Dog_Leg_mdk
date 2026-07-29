#include "app_chassis.h"

#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "stdint.h"
#include "RM_Lib.h"
#include "CP_System.h"
#include "DM.h"
#include "leg.h"
#include "packet.h"
#include "imu_data_decode.h"
#include <math.h>
#include "hipnuc_dec.h"

/* USER CODE BEGIN PTD */
// 定义私有类型（Private TypeDef），如结构体、枚举�?�联合体
/*总功率限制与功率再分配↓*/
typedef float f;
typedef uint8_t u8;
typedef int16_t i16;
typedef struct
{
  float PID_1_out;
  float PID_2_out;
  float PID_3_out;
  float PID_4_out;
  float PID_min_out;
  float Klimit;
  float limit_output;
  float TotalOutput;
  float KlimitGain;
  float used_power;
} DP_Power_Limit_t;

typedef struct
{
  uint32_t tick_ms;
  float pitch;
  float pitch_rate;
  float roll;
  float roll_rate;
  float left_angle;
  float right_angle;
  float left_speed;
  float right_speed;
  float left_feedback_torque;
  float right_feedback_torque;
  float forward_cmd;
  float forward_accel;
  float wheel_stall_ratio;
  float smc_cmd;
  float accel_ff_cmd;
  float height_damping_cmd;
  float height_hold_cmd;
  float roll_balance_cmd;
  float height;
  float height_speed;
  float left_cmd;
  float right_cmd;
  // 0.01Nm/LSB。使用int16而不是float，512点缓冲只增加4KiB。
  int16_t left_raw_cmd_cnm;
  int16_t right_raw_cmd_cnm;
  int16_t left_stage_cmd_cnm;
  int16_t right_stage_cmd_cnm;
  uint8_t state;
} LegTraceSample;
struct DP_Motor_FP
{
  u8 M3508_Flag;
  u8 DM_Flag;
  u8 TIM3_Flag;
  DP_Motor_FP() : M3508_Flag(0), DM_Flag(0), TIM3_Flag(0) {}
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 定义私有宏（Private Define），如常量�?�配置参�??
#define CP_SYSTEM_USART_HANDLE huart3
#define HI266_USART huart1
#define CHASSIC_TOTAL_OUTPUT_MAX 48000

#define SuperCap_ON HAL_GPIO_WritePin(HEN_GPIO_Port, HEN_Pin, GPIO_PIN_SET);
#define SuperCap_OFF HAL_GPIO_WritePin(HEN_GPIO_Port, HEN_Pin, GPIO_PIN_RESET);

#define Motor_Yaw_front -2.53901f // 弧度
#define rad_T 180.0f / 3.1415f

#define YAW_ERROR_MASK (0x0001 << 0)
#define YAW_BACK_FLAG_MASK (0x0001 << 1)
#define BUFF_FLAG_MASK (0x0001 << 2)

#define FRAME_SIZE 50
#define PI 3.1415926

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 定义私有宏函数或复杂宏（Private Macro�??
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 声明私有全局变量（Private Variables），如模块状态�?�缓冲区
DBUS YK(&huart6);

USER_CAN CAN_Motor(&hcan1, 0);
USER_CAN CAN_Communicate(&hcan2, 1);

MOTOR_RM M3508_MOTOR_QZ(0x201, &CAN_Motor),
    M3508_MOTOR_HZ(0x202, &CAN_Motor),
    M3508_MOTOR_HY(0x203, &CAN_Motor),
    M3508_MOTOR_QY(0x204, &CAN_Motor);
MOTOR_DM Left_Leg(0x12, &CAN_Motor),
    Right_Leg(0x11, &CAN_Motor),
    YAW(0x12, &CAN_Communicate);
PID_class PID_Chassis(7, 0, 0, 350, 0, 0, 500);

PID_class PID_Chassis_FeiPo(20, 0, 10, 500, 0, 200, 500);

PID_class M3508_MOTOR_QZ_sp(5, 0, 0, 13000, 0, 0, 13000),
    M3508_MOTOR_HZ_sp(5, 0, 0, 13000, 0, 0, 13000),
    M3508_MOTOR_HY_sp(5, 0, 0, 13000, 0, 0, 13000),
    M3508_MOTOR_QY_sp(5, 0, 0, 13000, 0, 0, 13000);

MOTOR_DiPan DP;
BMI088 CHASSIS_088(&hspi1, &htim7, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7, 3000, 1.7f, BMI088_GYRO_RANGE_2000, BMI088_ACC_RANGE_24);

DP_Power_Limit_t DP_Power_Limit;
DP_Motor_FP Motor_Flag;

UpDown_check_class UD_E(0), UD_Q(0), UD_Chase(0), UD_Speed_Cut(0), UD_XTL(0), UD_SpeedUp(0), UD_SpeedDown(0), UD_FeiPo(0), UD_yaogan(0);
UpDown_check_class UD_MID(0);
uint8_t UD_E_buf, UD_Q_buf,UD_Mid_buf;
uint8_t CHASSIS_088_State;

uint32_t M3508_1 = 0, M3508_2 = 0, M3508_3 = 0, M3508_4 = 0; // 测试电机�??
uint16_t Chase_Flag, Chase_Time;                             // 追杀标志位和计时

uint8_t YAW_Error = 1;
float Yaw_goal;
uint8_t Yaw_Back;
f XTL_PID_OUT;
/*CAN通讯接收云台板发送的数据�??*/
float what, why;
uint16_t Flag;
float Gimbal_Roll, Gimbal_Pitch, Gimbal_Roll_Acc, Gimbal_Pitch_Acc;
static uint32_t Gimbal_Roll_Tick, Gimbal_Pitch_Tick;
static uint32_t Gimbal_Roll_Acc_Tick, Gimbal_Pitch_Acc_Tick;
float V_Bat, V_Cap, V_Load;
static u8 v_can_buff;
float V_Bat_Real, V_Cap_Real, V_Load_Real;
float hi_yaw, hi_pitch, hi_roll;
uint8_t rx_buffer[FRAME_SIZE];
/*CAN通讯接收数控板发送的数据�??*/
/*标志�??*/
uint8_t Doghole_Flag = 0, Follow_Flag = 0, XTL_Flag = 0, Reset_flag = 0; // 狗洞、跟随�?�小�??螺�?�复位标志位
uint16_t speed = 0, XTL_speed = 0, Speed_Cut_Flag = 0;  // 初始速度以及小陀螺�?�度变量
int16_t SpeedChange = 0;
f XTL_SPEED_OUT = 0;
uint16_t XTL_SPEED_FLAG = 8;

uint8_t Chassic_3508_Flag; // 3508状�?�检测标志位

float FB_Speed, LR_Speed, FB_Real_Speed, LR_Real_Speed, Target_Speed; // 用作斜坡函数
static int16_t yaw_relative_mang;                                     // 小陀螺解�??

uint16_t Communicate_Send_Flag_1, Communicate_Rx_Flag_1;

uint8_t UI_step = 1, UI_step_ten = 1, UI_change = 1, speedlimit_flag = 0; // UI相关数据
uint8_t play_mode;
static bool cp_state;
/*标志�??*/

int16_t Driver_OUT_PID, MCL_SPEED;
static float theta_angle, theta_rad; // theta角度和弧�??
static float sin_theta, cos_theta;
uint8_t Buff_Flag;
uint16_t E_Char[5] = {Color_Yellow, 50, 5, 850, 150};
uint16_t Q_Char[5] = {Color_Yellow, 50, 5, 700, 150};
uint16_t X_Char[5] = {Color_Green,50,5,630,600};
uint16_t MID_Char[5] = {Color_Green,40,4,900,850};
uint8_t E_Flag, Q_Flag,X_Flag, MID_Flag;
uint8_t YK_Mode, Leg_Mode;
uint8_t BMI088_Mode;
static u8 shaobin_state;
static u8 mid_state;
// INFO//
// Vision_LPF IMU_OUT(20,0.001,PI);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
// 定义私有函数原型（Private Function Prototypes�??
void Slow(float *rec, float target, float slow_Inc) // 斜坡底部函数
{
  if (abs(*rec) - abs(target) < 0)
  {
  }
  if (abs(*rec) - abs(target) > 0)
    slow_Inc = slow_Inc * 8; // 减�?�时放大8�??

  if (abs(*rec - target) < slow_Inc)
    *rec = target;
  else
  {
    if ((*rec) > target)
    {
      (*rec) -= slow_Inc;
      //			power_mode=1;
    }
    if ((*rec) < target)
      (*rec) += slow_Inc;
  }
}

float KEY_FB_Ctrl(float speed) // 键盘控制前后FB�??
{
  float res;
  Slow(&FB_Speed,
       speed * (YK.Pressed_Check(KEY_PRESSED_W) - YK.Pressed_Check(KEY_PRESSED_S)),
       20);
  res = FB_Speed;
  return res;
}
float KEY_LR_Ctrl(float speed) // 键盘控制左右，LR�??
{
  float res;

  Slow(&LR_Speed,
       speed * (YK.Pressed_Check(KEY_PRESSED_D) - YK.Pressed_Check(KEY_PRESSED_A)),
       20);
  res = LR_Speed;
  return res;
}
void KEY_Forback_Ctrl(float *rec1, float *rec2, float speed) // 键盘总控，前后左�??
{
  (*rec1) = KEY_FB_Ctrl(speed);
  (*rec2) = KEY_LR_Ctrl(speed);
}
void F_slow(float *in, float target, float add_inc, float cut_inc, float stop_err) // 逐步调节 in 指向的�?�，使其逐渐接近 target 指向的�?�，add为加速增量�??
{
  if (abs(*in - target) <= stop_err)
    *in = target;
  else
  {
    float in_buf = 0;
    if (*in < target)
    {
      in_buf = *in + add_inc;

      if (in_buf > target)
      {
        *in = target;
      }
      else
      {
        *in = in_buf;
      }
    }
    else
    {
      in_buf = *in - cut_inc;
      *in -= cut_inc;
      if (in_buf < target)
      {
        *in = target;
      }
      else
      {
        *in = in_buf;
      }
    }
  }
}
float fb_add_sp = 1, fb_cut_sp = 2.5, lr_add_sp = 1, lr_cut_sp = 2.5;
static float Chassic_Ch0_Real, Chassic_Ch1_Real, Chassic_Ch2_Real;
static float Leg_Forward_Accel;
static float Leg_Forward_Accel_State;
static float Leg_Forward_Last_Cmd;
static uint8_t Leg_Forward_State_Valid;
static uint8_t Leg_Forward_Last_Mode;
static uint8_t Leg_Forward_Last_Xtl;

#define LEG_TRACE_SAMPLE_COUNT 512U
volatile LegTraceSample leg_trace_buffer[LEG_TRACE_SAMPLE_COUNT];
volatile uint16_t leg_trace_write_index;
volatile uint16_t leg_trace_valid_count;
volatile uint16_t leg_trace_post_remaining;
volatile uint8_t leg_trace_armed = 1;
volatile uint8_t leg_trace_triggered;
volatile uint8_t leg_trace_frozen;
volatile uint8_t leg_trace_trigger_request;
volatile uint8_t leg_trace_rearm_request;

static void Leg_Trace_Update(const LegControlInput *input,
                             const LegControlOutput *output)
{
  static uint8_t divider;
  static uint8_t last_state;

  if (leg_trace_rearm_request)
  {
    leg_trace_write_index = 0;
    leg_trace_valid_count = 0;
    leg_trace_post_remaining = 0;
    leg_trace_triggered = 0;
    leg_trace_frozen = 0;
    leg_trace_armed = 1;
    leg_trace_rearm_request = 0;
    divider = 0;
  }

  if (leg_trace_frozen || ++divider < 8)
    return;
  divider = 0;

  uint16_t index = leg_trace_write_index;
  volatile LegTraceSample *sample = &leg_trace_buffer[index];
  sample->tick_ms = HAL_GetTick();
  sample->pitch = input->gimbal_roll;
  sample->pitch_rate = input->gimbal_roll_acc;
  sample->roll = input->gimbal_pitch;
  sample->roll_rate = input->gimbal_pitch_acc;
  sample->left_angle = input->left_motor->mang;
  sample->right_angle = input->right_motor->mang;
  sample->left_speed = input->left_motor->sp;
  sample->right_speed = input->right_motor->sp;
  sample->left_feedback_torque = input->left_motor->Torque;
  sample->right_feedback_torque = input->right_motor->Torque;
  sample->forward_cmd = input->forward_cmd;
  sample->forward_accel = input->forward_accel;
  sample->wheel_stall_ratio = input->wheel_stall_ratio;
  sample->smc_cmd = output->smc_cmd;
  sample->accel_ff_cmd = output->accel_ff_cmd;
  sample->height_damping_cmd = output->height_damping_cmd;
  sample->height_hold_cmd = output->height_hold_cmd;
  sample->roll_balance_cmd = output->roll_balance_cmd;
  sample->height = output->height;
  sample->height_speed = output->height_speed;
  sample->left_cmd = output->left_torque;
  sample->right_cmd = output->right_torque;
  sample->left_raw_cmd_cnm =
      (int16_t)(LIMIT(output->left_raw_torque, -327.0f, 327.0f) * 100.0f);
  sample->right_raw_cmd_cnm =
      (int16_t)(LIMIT(output->right_raw_torque, -327.0f, 327.0f) * 100.0f);
  sample->left_stage_cmd_cnm =
      (int16_t)(LIMIT(output->left_stage_torque, -327.0f, 327.0f) * 100.0f);
  sample->right_stage_cmd_cnm =
      (int16_t)(LIMIT(output->right_stage_torque, -327.0f, 327.0f) * 100.0f);
  sample->state = output->dynamic_state;

  leg_trace_write_index = (uint16_t)((index + 1U) % LEG_TRACE_SAMPLE_COUNT);
  if (leg_trace_valid_count < LEG_TRACE_SAMPLE_COUNT)
    leg_trace_valid_count++;

  uint8_t automatic_trigger =
      (output->dynamic_state == LEG_DYNAMIC_SUPPORT_HOLD ||
       output->dynamic_state == LEG_DYNAMIC_UNLOAD_CATCH) &&
      last_state != output->dynamic_state;
  if (leg_trace_armed && (leg_trace_trigger_request || automatic_trigger ||
                          fabsf(input->forward_accel) > 2.0f))
  {
    leg_trace_triggered = 1;
    leg_trace_armed = 0;
    leg_trace_post_remaining = LEG_TRACE_SAMPLE_COUNT / 2U;
    leg_trace_trigger_request = 0;
  }

  if (leg_trace_triggered && leg_trace_post_remaining > 0U)
  {
    leg_trace_post_remaining--;
    if (leg_trace_post_remaining == 0U)
      leg_trace_frozen = 1;
  }
  last_state = output->dynamic_state;
}

static float Leg_Approach(float now, float target, float step)
{
  if (now < target)
    return (now + step > target) ? target : now + step;
  if (now > target)
    return (now - step < target) ? target : now - step;
  return now;
}

// Jerk-limited replacement for the existing forward speed ramp. The maximum
// acceleration remains 1.0/2.5 command units per TIM3 period.
static void Leg_Forward_SCurve(float *value, float target)
{
  float desired_accel;
  float error = target - *value;
  float accel_limit = (fabsf(target) > fabsf(*value) && target * (*value) >= 0.0f) ?
                          fb_add_sp : fb_cut_sp;

  if (fabsf(error) < 0.001f)
    desired_accel = 0.0f;
  else
    desired_accel = (error > 0.0f) ? accel_limit : -accel_limit;

  // Reach the requested acceleration in about 20 ms (10 TIM3 periods).
  Leg_Forward_Accel_State = Leg_Approach(Leg_Forward_Accel_State,
                                         desired_accel,
                                         accel_limit / 10.0f);
  if (fabsf(error) <= fabsf(Leg_Forward_Accel_State))
  {
    *value = target;
    Leg_Forward_Accel_State = 0.0f;
  }
  else
  {
    *value += Leg_Forward_Accel_State;
  }
}
static void Chassic_axis_slow(float *real, float target, float add_sp, float cut_sp)
{
  if (*real > 0)
  {
    F_slow(real, target, add_sp, cut_sp, cut_sp);
  }
  else
  {
    F_slow(real, target, cut_sp, add_sp, cut_sp);
  }
}
void KEY_Forback_Ctrl1(void)
{
  FB_Speed = (YK.Pressed_Check(KEY_PRESSED_W) - YK.Pressed_Check(KEY_PRESSED_S)) * Target_Speed;
  if (XTL_Flag)
  {
    LR_Speed = (YK.Pressed_Check(KEY_PRESSED_D) - YK.Pressed_Check(KEY_PRESSED_A)) * Target_Speed;
  }
  else
  {
    // LR_Speed = (YK.Pressed_Check(KEY_PRESSED_D) - YK.Pressed_Check(KEY_PRESSED_A)) * Target_Speed / 2.0f;
    LR_Speed = (YK.Pressed_Check(KEY_PRESSED_D) - YK.Pressed_Check(KEY_PRESSED_A)) * Target_Speed;
  }

  if (leg_enable_forward_jerk_limit)
  {
    Leg_Forward_SCurve(&FB_Real_Speed, FB_Speed);
  }
  else if (FB_Real_Speed > 0)
  {
    F_slow(&FB_Real_Speed, FB_Speed, fb_add_sp, fb_cut_sp, fb_cut_sp);
  }
  else
  {
    F_slow(&FB_Real_Speed, FB_Speed, fb_cut_sp, fb_add_sp, fb_cut_sp);
  }
  if (LR_Real_Speed > 0)
  {
    F_slow(&LR_Real_Speed, LR_Speed, lr_add_sp, lr_cut_sp, lr_cut_sp);
  }
  else
  {
    F_slow(&LR_Real_Speed, LR_Speed, lr_cut_sp, lr_add_sp, lr_cut_sp);
  }
}

// Preserve the coordinate selection used by the earlier PID feedforward code.
static float Leg_Get_Forward_Command(void)
{
  if (YK_Mode == PLAYER_MODE)
  {
    float player_fb = FB_Real_Speed;
    if (XTL_Flag)
    {
      player_fb = FB_Real_Speed * cos_theta + LR_Real_Speed * sin_theta;
    }
    // PLAYER 的第二个底盘运动分量与 CONTROL 的 Chassic_Ch1_Real
    // 是同一物理前进轴；两种模式必须使用相同的负号约定。横向运动
    // 不再伪装成前进加速度，避免给 pitch 腿部前馈注入错误方向/错误轴。
    return (fabsf(player_fb) > 120.0f) ? -player_fb : 0.0f;
  }
  if (YK_Mode == CONTROL_MODE)
    return (fabsf(Chassic_Ch1_Real) > 120.0f) ? -Chassic_Ch1_Real : 0.0f;
  if (YK_Mode == XTL_MODE)
    return Chassic_Ch0_Real;
  return 0.0f;
}

static void Leg_Update_Forward_State(float dt)
{
  float cmd = Leg_Get_Forward_Command();
  if (!Leg_Forward_State_Valid || Leg_Forward_Last_Mode != YK_Mode ||
      Leg_Forward_Last_Xtl != XTL_Flag)
  {
    Leg_Forward_Last_Cmd = cmd;
    Leg_Forward_Accel = 0.0f;
    Leg_Forward_State_Valid = 1;
    Leg_Forward_Last_Mode = YK_Mode;
    Leg_Forward_Last_Xtl = XTL_Flag;
    return;
  }

  // Express acceleration as command increment per legacy 4 ms leg-control step.
  Leg_Forward_Accel = (cmd - Leg_Forward_Last_Cmd) * (0.004f / dt);
  Leg_Forward_Last_Cmd = cmd;
}
void Chassic_Forback_Ctrl(void)
{
  float ch0_target = 0;
  float ch1_target = 0;
  float ch2_target = 0;

  if (YK_Mode == ONLY_CHASSIC || YK_Mode == FAST_CHASSIC || YK_Mode == CONTROL_MODE || YK_Mode == XTL_MODE)
  {
    ch0_target = YK.yaogan.ch0;
    ch1_target = YK.yaogan.ch1;
    ch2_target = YK.yaogan.ch2;
  }

  Chassic_axis_slow(&Chassic_Ch0_Real, ch0_target, fb_add_sp, fb_cut_sp);
  Chassic_axis_slow(&Chassic_Ch1_Real, ch1_target, lr_add_sp, lr_cut_sp);
  Chassic_axis_slow(&Chassic_Ch2_Real, ch2_target, lr_add_sp, lr_cut_sp);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MODE_DEAL()
{
  if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_UP)
    YK_Mode = PROTECT_MODE; // 双上，保护模�??
  else if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_MID)
    YK_Mode = ONLY_GIMBAL; // 上中，只控制云台
  else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_UP)
    YK_Mode = ONLY_CHASSIC; // 中上，只控制底盘
  else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_MID)
    YK_Mode = CONTROL_MODE; // 双中，控制模�??
  else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_MID)
    YK_Mode = XTL_MODE; // 下中，小�??螺模�??
  else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_DOWN)
    YK_Mode = SHOOT_MODE; // 中下，射击模�??
  else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_DOWN)
    YK_Mode = PLAYER_MODE; // 双下，�?�手端模�??
  else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_UP)
    YK_Mode = FAST_CHASSIC; // 下上，快速底盘模�??
  else
    YK_Mode = PROTECT_MODE;

  if (YK_Mode == ONLY_CHASSIC && YK.yaogan.v < 600)
  {
    Leg_Mode = GYRO_MODE;
  }
  else if (YK_Mode == ONLY_CHASSIC && YK.yaogan.v > 600)
  {
    play_mode ^= (0x01 << 1);
  }
  else if (YK_Mode == FAST_CHASSIC)
  {
    Leg_Mode = AUTO_MODE;
  }
  else if (YK_Mode == CONTROL_MODE || YK_Mode == PLAYER_MODE)
  {
    Leg_Mode = Leg_Keep_Mode;
  }
  else
  {
    Leg_Mode = PROTECT_MODE;
  }
}
void M3508_Limit_deal()
{
  if (YK_Mode == PROTECT_MODE || YK_Mode == SHOOT_MODE)
  {
    CAN_Motor.Send_RM(0x200, 0, 0, 0, 0);
  }
  else
  {
    DP_Power_Limit.Klimit = (float)(V_Cap_Real / 20.0f);
    // 键盘控制
    switch (v_can_buff)
    {
      case 0:
      DP_Power_Limit.KlimitGain = 1;
      V_Cap_Real = 0.0f;
      break;
      
      case 1:
        if (V_Cap_Real >= 12.0)
    {
      DP_Power_Limit.KlimitGain = 1;
    }
    else if (V_Cap_Real >= 10.0 && V_Cap_Real < 12.0)
    {
      DP_Power_Limit.KlimitGain = DP_Power_Limit.Klimit;
      if (YK.Pressed_Check(KEY_PRESSED_SHIFT))
        DP_Power_Limit.KlimitGain = DP_Power_Limit.Klimit * 1.5; // 系数根据实际情况调整
    }
    else if (V_Cap_Real >= 8.0 && V_Cap_Real < 10.0)
    {
      DP_Power_Limit.KlimitGain = (DP_Power_Limit.Klimit * DP_Power_Limit.Klimit) * 5 / 4;
      if (YK.Pressed_Check(KEY_PRESSED_SHIFT))
        DP_Power_Limit.KlimitGain = DP_Power_Limit.Klimit * DP_Power_Limit.Klimit * 1.5;
    }
    else if (V_Cap_Real < 8.0)
    {
      DP_Power_Limit.KlimitGain = DP_Power_Limit.Klimit * DP_Power_Limit.Klimit * DP_Power_Limit.Klimit;
      if (YK.Pressed_Check(KEY_PRESSED_SHIFT))
        DP_Power_Limit.KlimitGain = DP_Power_Limit.Klimit * DP_Power_Limit.Klimit * DP_Power_Limit.Klimit * 1.5;
    }
      break;
    }
    // if (V_Cap_Real == 0)
    //   DP_Power_Limit.KlimitGain = 1;

    DP_Power_Limit.limit_output = DP_Power_Limit.KlimitGain * CHASSIC_TOTAL_OUTPUT_MAX;
    DP_Power_Limit.TotalOutput = fabs(M3508_MOTOR_QZ_sp.OUT_PID) + fabs(M3508_MOTOR_HZ_sp.OUT_PID) + fabs(M3508_MOTOR_HY_sp.OUT_PID) + fabs(M3508_MOTOR_QY_sp.OUT_PID);

    if (DP_Power_Limit.TotalOutput >= DP_Power_Limit.limit_output) // 功率限制
    {
      if (DP_Power_Limit.PID_min_out > fabs(M3508_MOTOR_QZ_sp.OUT_PID))
        DP_Power_Limit.PID_min_out = fabs(M3508_MOTOR_QZ_sp.OUT_PID);
      if (DP_Power_Limit.PID_min_out > fabs(M3508_MOTOR_HZ_sp.OUT_PID))
        DP_Power_Limit.PID_min_out = fabs(M3508_MOTOR_HZ_sp.OUT_PID);
      if (DP_Power_Limit.PID_min_out > fabs(M3508_MOTOR_HY_sp.OUT_PID))
        DP_Power_Limit.PID_min_out = fabs(M3508_MOTOR_HY_sp.OUT_PID);
      if (DP_Power_Limit.PID_min_out > fabs(M3508_MOTOR_QY_sp.OUT_PID))
        DP_Power_Limit.PID_min_out = fabs(M3508_MOTOR_QY_sp.OUT_PID);

      //						M3508_MOTOR_QZ_sp.LIMIT_PID = DP_Power_Limit.PID_min_out - 10;
      //						M3508_MOTOR_HZ_sp.LIMIT_PID = DP_Power_Limit.PID_min_out - 10;
      //						M3508_MOTOR_HY_sp.LIMIT_PID = DP_Power_Limit.PID_min_out - 10;
      //						M3508_MOTOR_QY_sp.LIMIT_PID = DP_Power_Limit.PID_min_out - 10;

      DP_Power_Limit.PID_1_out = M3508_MOTOR_QZ_sp.OUT_PID / DP_Power_Limit.TotalOutput * DP_Power_Limit.limit_output;
      DP_Power_Limit.PID_2_out = M3508_MOTOR_HZ_sp.OUT_PID / DP_Power_Limit.TotalOutput * DP_Power_Limit.limit_output;
      DP_Power_Limit.PID_3_out = M3508_MOTOR_HY_sp.OUT_PID / DP_Power_Limit.TotalOutput * DP_Power_Limit.limit_output;
      DP_Power_Limit.PID_4_out = M3508_MOTOR_QY_sp.OUT_PID / DP_Power_Limit.TotalOutput * DP_Power_Limit.limit_output;
    }
    else
    {
      M3508_MOTOR_QZ_sp.LIMIT_PID = 12000;
      M3508_MOTOR_HZ_sp.LIMIT_PID = 12000;
      M3508_MOTOR_HY_sp.LIMIT_PID = 12000;
      M3508_MOTOR_QY_sp.LIMIT_PID = 12000;

      DP_Power_Limit.PID_1_out = M3508_MOTOR_QZ_sp.OUT_PID;
      DP_Power_Limit.PID_2_out = M3508_MOTOR_HZ_sp.OUT_PID;
      DP_Power_Limit.PID_3_out = M3508_MOTOR_HY_sp.OUT_PID;
      DP_Power_Limit.PID_4_out = M3508_MOTOR_QY_sp.OUT_PID;
    }
    CAN_Motor.Send_RM(0x200, DP_Power_Limit.PID_1_out, DP_Power_Limit.PID_2_out, DP_Power_Limit.PID_3_out, DP_Power_Limit.PID_4_out);
  }
}
void Communicate_deal()
{
  if (CAN_Communicate.RxHeader.StdId == 0x110) // 接收云台板发送的遥控器数据和云台状�??
    YK.can_receive_data_deal(CAN_Communicate.rx_buf);
  if (CAN_Communicate.RxHeader.StdId == 0x113)
  {
    YK.V_can_receive_data_deal(CAN_Communicate.rx_buf);
    Communicate_Rx_Flag_1 = CAN_Communicate.rx_buf[3] | CAN_Communicate.rx_buf[2] << 8;
    Driver_OUT_PID = CAN_Communicate.rx_buf[5] | CAN_Communicate.rx_buf[4] << 8;
    MCL_SPEED = CAN_Communicate.rx_buf[7] | CAN_Communicate.rx_buf[6] << 8;
  }
  if (CAN_Communicate.RxHeader.StdId == 0x123)
  {
    union
    {
      float f;
      uint8_t c[4];
    } angle;
    angle.c[0] = CAN_Communicate.rx_buf[1];
    angle.c[1] = CAN_Communicate.rx_buf[2];
    angle.c[2] = CAN_Communicate.rx_buf[3];
    angle.c[3] = CAN_Communicate.rx_buf[4];
    uint32_t now_tick = HAL_GetTick();
    if (CAN_Communicate.rx_buf[0] == 0)
    {
      Gimbal_Roll = angle.f;
      Gimbal_Roll_Tick = now_tick;
    }
    else if (CAN_Communicate.rx_buf[0] == 1)
    {
      Gimbal_Pitch = angle.f;
      Gimbal_Pitch_Tick = now_tick;
    }
    else if (CAN_Communicate.rx_buf[0] == 2)
    {
      Gimbal_Roll_Acc = angle.f;
      Gimbal_Roll_Acc_Tick = now_tick;
    }
    else if (CAN_Communicate.rx_buf[0] == 3)
    {
      Gimbal_Pitch_Acc = angle.f;
      Gimbal_Pitch_Acc_Tick = now_tick;
    }
  }
  if (CAN_Communicate.RxHeader.StdId == 0x120) // 接收数控板发送的数据，包括电容电�??
  {
    V_Bat = CAN_Communicate.rx_buf[1] | CAN_Communicate.rx_buf[0] << 8;
    V_Cap = CAN_Communicate.rx_buf[3] | CAN_Communicate.rx_buf[2] << 8;
    V_Load = CAN_Communicate.rx_buf[5] | CAN_Communicate.rx_buf[4] << 8;
    v_can_buff = CAN_Communicate.rx_buf[7] | CAN_Communicate.rx_buf[6] << 8;
    V_Bat_Real = V_Bat / 100;
    V_Cap_Real = V_Cap / 100;
    V_Load_Real = V_Load / 100;
  }
}
void XTLkeyboarddeal(void)
{
	if (UD_XTL.updata(YK.Pressed_Check(KEY_PRESSED_Q)) == UpDown_check_rising)
	{
		if (XTL_Flag == 0)
		{
			XTL_Flag = 1;
			// N_auto_flag = 0;
			// N_auto_xtl_flag=0; 
			Communicate_Send_Flag_1 &= ~(0x0001 << 4);
			Communicate_Send_Flag_1 |= (0x0001 << 3);
			// XTL_SPEED_OUT = XTL_speed;
		}
		else
		{
			XTL_Flag = 0;
			XTL_SPEED_OUT = 0;
			SpeedChange = 0;
			XTL_PID_OUT = 0;
			Communicate_Send_Flag_1 &= ~(0x0001 << 3);
		}
	}

  f xtl_speed_target = (XTL_Flag || YK_Mode == XTL_MODE) ? XTL_speed : 0;
	Slow(&XTL_SPEED_OUT, xtl_speed_target, 10);
	UD_Q_buf = UD_Q.updata(XTL_Flag);
	if (UD_Q_buf == UpDown_check_rising)
	{
		Q_Flag = 1;
		Q_Char[0] = Color_Pink;
	}
	else if (UD_Q_buf == UpDown_check_falling)
	{
		Q_Flag = 1;
		Q_Char[0] = Color_Yellow;
	}
}
void chassic_power_deal(void)
{
	if (CP.ext_game_robot_status_t.chassis_power_limit > 1 && CP.ext_game_robot_status_t.chassis_power_limit < 45)
	{
		speed = 200 + SpeedChange;
		XTL_speed = 250 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 45 && CP.ext_game_robot_status_t.chassis_power_limit < 50)
	{
		speed = 230 + SpeedChange;
		XTL_speed = 280 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 50 && CP.ext_game_robot_status_t.chassis_power_limit < 55)
	{
		speed = 260 + SpeedChange;
		XTL_speed = 300 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 55 && CP.ext_game_robot_status_t.chassis_power_limit < 60)
	{
		speed = 300 + SpeedChange;
		XTL_speed = 340 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 60 && CP.ext_game_robot_status_t.chassis_power_limit < 65)
	{
		speed = 330 + SpeedChange;
		XTL_speed = 360 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 65 && CP.ext_game_robot_status_t.chassis_power_limit < 70)
	{
		speed = 360 + SpeedChange;
		XTL_speed = 400 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 70 && CP.ext_game_robot_status_t.chassis_power_limit < 75)
	{
		speed = 390 + SpeedChange;
		XTL_speed = 440 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 75 && CP.ext_game_robot_status_t.chassis_power_limit < 80)
	{
		speed = 400 + SpeedChange;
		XTL_speed = 450 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 80 && CP.ext_game_robot_status_t.chassis_power_limit < 85)
	{
		speed = 425 + SpeedChange;
		XTL_speed = 450 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 85 && CP.ext_game_robot_status_t.chassis_power_limit < 90)
	{
		speed = 440 + SpeedChange;
		XTL_speed = 470 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 90 && CP.ext_game_robot_status_t.chassis_power_limit < 95)
	{
		speed = 460 + SpeedChange;
		XTL_speed = 480 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 95 && CP.ext_game_robot_status_t.chassis_power_limit < 100)
	{
		speed = 470 + SpeedChange;
		XTL_speed = 480 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 100 && CP.ext_game_robot_status_t.chassis_power_limit < 120)
	{
		speed = 480 + SpeedChange;
		XTL_speed = 490 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 120 && CP.ext_game_robot_status_t.chassis_power_limit < 200)
	{
		speed = 500 + SpeedChange;
		XTL_speed = 500 + SpeedChange;
	}
	else if (CP.ext_game_robot_status_t.chassis_power_limit >= 200)
	{
		speed = 500 + SpeedChange;
		XTL_speed = 520 + SpeedChange;
	}
	else
	{
		speed = 550 + SpeedChange;
		XTL_speed = 500 + SpeedChange;
	}
}
void PLAYER_deal()
{
  if (YK.Pressed_Check(KEY_PRESSED_SHIFT))
  {
    Target_Speed = speed + 200 + XTL_Flag * (CP.ext_game_robot_status_t.chassis_power_limit - 20) * 1.2;
    YK.yaogan.ch1 = LR_Real_Speed;
    YK.yaogan.ch0 = FB_Real_Speed;
  }
  else // 键盘移动
  {
    Target_Speed = speed + XTL_Flag * (CP.ext_game_robot_status_t.chassis_power_limit - 20) * 0.4;
    YK.yaogan.ch1 = LR_Real_Speed;
    YK.yaogan.ch0 = FB_Real_Speed;
  }
  if (YAW_Error == 0)
  {
    if (XTL_Flag)
    {
      DP.ML_Data_Deal(LR_Real_Speed * cos_theta - FB_Real_Speed * sin_theta, FB_Real_Speed * cos_theta + LR_Real_Speed * sin_theta, XTL_SPEED_OUT + 0.5f * ((fabs(-LR_Real_Speed) < fabs(-FB_Real_Speed)) ? fabs(-LR_Real_Speed) : fabs(-FB_Real_Speed)), 8000);
    }
    else if (Yaw_Back == 1)
    {
      DP.ML_Data_Deal(LR_Real_Speed * cos_theta - FB_Real_Speed * sin_theta, FB_Real_Speed * cos_theta + LR_Real_Speed * sin_theta, 0, 8000);
    }
    else
    {
      DP.ML_Data_Deal(LR_Real_Speed * cos_theta - FB_Real_Speed * sin_theta, FB_Real_Speed * cos_theta + LR_Real_Speed * sin_theta, PID_Chassis.OUT_PID, 8000);
    }
  }
}

void Key_deal()
{
  chassic_power_deal(); // 底盘功率处理
  if(YAW.mang < Motor_Yaw_front + 1.0f && YAW.mang > Motor_Yaw_front - 1.0f)
  {
    mid_state = 1;
  }
  else mid_state = 0;

  if (Communicate_Rx_Flag_1 & YAW_BACK_FLAG_MASK)
    Yaw_Back = 1;
  else
    Yaw_Back = 0;

  if (Communicate_Rx_Flag_1 & BUFF_FLAG_MASK)
  {
    Buff_Flag = 1;
  }
  else
  {
    Buff_Flag = 0;
  }
  UD_Mid_buf = UD_MID.updata(mid_state);
  if (UD_Mid_buf == UpDown_check_rising)
  {
    MID_Flag = 1;
    MID_Char[0] = Color_Green;
  }
  else if (UD_Mid_buf == UpDown_check_falling)
  {
    MID_Flag = 1;
    MID_Char[0] = Color_White;
  }
  if (CP.ext_game_robot_status_t.robot_id < 10)
    Communicate_Send_Flag_1 |= (0x0001 << 0); // 发�?�标�??
  else
    Communicate_Send_Flag_1 &= ~(0x0001 << 0);

  if (Buff_Flag != 0)
    Communicate_Send_Flag_1 |= (0x0001 << 2);
  else
    Communicate_Send_Flag_1 &= ~(0x0001 << 2);

  if (UD_Chase.updata(YK.shubiao.press_l) == UpDown_check_rising || YK.shubiao.press_l) // ׷ɱ
  {
    Chase_Flag = 1;
    Chase_Time = 0;
  }
  if (UD_yaogan.updata(YK.yaogan.v > 600) == UpDown_check_rising && YK_Mode == XTL_MODE)
	{
		XTL_SPEED_FLAG += 2;
		if (XTL_SPEED_FLAG >= 20)
			XTL_SPEED_FLAG = 8;
    SpeedChange = (XTL_SPEED_FLAG - 8) * 50;
	}
	if (CP.ext_game_robot_status_t.robot_id < 10)
		Communicate_Send_Flag_1 |= (0x0001 << 0);
	else
		Communicate_Send_Flag_1 &= ~(0x0001 << 0);

	if (UD_Chase.updata(YK.shubiao.press_l) == UpDown_check_rising || YK.shubiao.press_l) // 追杀
	{
		Chase_Flag = 1;
		Chase_Time = 0;
	}

	if (UD_SpeedUp.updata(YK.Pressed_Check(KEY_PRESSED_C)) == UpDown_check_rising && (YK.jianpan & KEY_PRESSED_CTRL))
	{
		if (SpeedChange < 500)
			SpeedChange += 100;
	}

	if (UD_SpeedDown.updata(YK.Pressed_Check(KEY_PRESSED_C)) == UpDown_check_rising && !(YK.jianpan & KEY_PRESSED_CTRL))
	{
		if (SpeedChange > -200)
			SpeedChange -= 100;
	}

  if (Leg_Control_Mode_Update(YK_Mode == PLAYER_MODE, YK.Pressed_Check(KEY_PRESSED_X)) ||
      (YK_Mode != PLAYER_MODE && X_Char[0] != Color_Green))
  {
    X_Char[0] = Leg_Control_Mode_Is_Enabled() ? Color_Red_Or_Bule : Color_Green;
    X_Flag = 1;
  }
  if(YK_Mode == PLAYER_MODE)
  {
    UD_E_buf = UD_E.updata(Buff_Flag);
    if (UD_E_buf == UpDown_check_rising)
    {
      E_Flag = 1;
      E_Char[0] = Color_Pink;
    }
    else if (UD_E_buf == UpDown_check_falling)
    {
      E_Flag = 1;
      E_Char[0] = Color_Yellow;
    }
  }


	XTLkeyboarddeal();
  if (YK.Pressed_Check(KEY_PRESSED_Z) && YK.Pressed_Check(KEY_PRESSED_CTRL))
  {
    uint8_t temp;
    Reset_flag = 1;
    HAL_Delay(10);
    for (temp = 0; temp < 20; temp++)
    {
      M3508_MOTOR_QZ_sp.PID_update(0, M3508_MOTOR_QZ.sp);
      M3508_MOTOR_HZ_sp.PID_update(0, M3508_MOTOR_HZ.sp);
      M3508_MOTOR_HY_sp.PID_update(0, M3508_MOTOR_HY.sp);
      M3508_MOTOR_QY_sp.PID_update(0, M3508_MOTOR_QY.sp);
      CAN_Motor.Send_RM(0x200, M3508_MOTOR_QZ_sp.OUT_PID, M3508_MOTOR_HZ_sp.OUT_PID, M3508_MOTOR_HY_sp.OUT_PID, M3508_MOTOR_QY_sp.OUT_PID);
      HAL_Delay(10);
    }
    CAN_Motor.Send_RM(0x200, 0, 0, 0, 0);
    HAL_Delay(100);

    __set_FAULTMASK(1); // 关闭�??有中�??
    NVIC_SystemReset(); // 复位
  }
}
/* USER CODE END 0 */

void App_Chassis_Init(void)
{
  HAL_Delay(2000);
  CAN_Motor.Init(0, 0);
  HAL_Delay(5);
  CAN_Communicate.Init(1, 1);
  // imu_data_decode_init();
  // test_flag();
  //IMU_UART_Init();
  Leg_Control_Config_MotorLimit(&Left_Leg, &Right_Leg);//设置6248参数 根据不同电机自行调整
  HAL_Delay(10);
  CP_System_Init();
  Left_Leg.DM_Start(0x02);
  HAL_Delay(20);
  Right_Leg.DM_Start(0x01);
  HAL_Delay(20);
  HAL_TIM_Base_Start_IT(&htim2); // UI绘制 25hZ
  HAL_TIM_Base_Start_IT(&htim5); // CAN通讯 100Hz
  HAL_TIM_Base_Start_IT(&htim3); // �??�??3508在线 1000Hz
  HAL_TIM_Base_Start_IT(&htim7);
  // IMU_OUT.Vision_Low_Pass_Filter_Init();
}

void App_Chassis_Loop(void)
{
    MODE_DEAL();
    YAW_Error = 0;
    Key_deal();
    cp_state = Judge_IF_DUM_Normal();
    // INFO("%.2f,%.2f,%.2f\n",gyr_x,gyr_y,gyr_z);
    // INFO("%.2f,%.2f,%.2f\n",Gimbal_Pitch,Right_Leg.mang*rad_T,Left_Leg.mang*rad_T);
}

void App_Chassis_CAN1_RxFifo0Callback(CAN_HandleTypeDef *hcan)
{
  if (CAN_Motor.Receive(&hcan1) == HAL_OK)
  {
    if (M3508_MOTOR_QZ.update() == HAL_OK)
    {
      M3508_1++;
      M3508_MOTOR_QZ_sp.PID_update(DP.ML.qz, M3508_MOTOR_QZ.sp);
      Chassic_3508_Flag |= 0x01;
    }
    if (M3508_MOTOR_HZ.update() == HAL_OK)
    {
      M3508_2++;
      M3508_MOTOR_HZ_sp.PID_update(DP.ML.hz, M3508_MOTOR_HZ.sp);
      Chassic_3508_Flag |= 0x02;
    }
    if (M3508_MOTOR_HY.update() == HAL_OK)
    {
      M3508_3++;
      M3508_MOTOR_HY_sp.PID_update(DP.ML.hy, M3508_MOTOR_HY.sp);
      Chassic_3508_Flag |= 0x04;
    }
    if (M3508_MOTOR_QY.update() == HAL_OK)
    {
      M3508_4++;
      M3508_MOTOR_QY_sp.PID_update(DP.ML.qy, M3508_MOTOR_QY.sp);
      Chassic_3508_Flag |= 0x08;
    }
    if (Left_Leg.DM_update() == HAL_OK)
    {
    }
    if (Right_Leg.DM_update() == HAL_OK)
    {
    }
  }
}
void App_Chassis_CAN2_RxFifo1Callback(CAN_HandleTypeDef *hcan)
{
  if (CAN_Communicate.Receive(&hcan2) == HAL_OK)
  {

    Communicate_deal();

    if (YAW.DM_update() == HAL_OK)
    {
      theta_rad = YAW.mang - Motor_Yaw_front;
      sin_theta = sinf(theta_rad);
      cos_theta = cosf(theta_rad);

      if (cos_theta > 0)
        PID_Chassis.PID_update(0, sin_theta * 70);
      else
        PID_Chassis.PID_update(0, -sin_theta * 70);
    }
    switch (YK_Mode)
      {
      case ONLY_CHASSIC:
      case FAST_CHASSIC:
        DP.ML_Data_Deal(Chassic_Ch0_Real, Chassic_Ch1_Real, Chassic_Ch2_Real, 8000);
        break;
      case CONTROL_MODE:
        DP.ML_Data_Deal(Chassic_Ch0_Real * cos_theta - Chassic_Ch1_Real * sin_theta,
                        Chassic_Ch1_Real * cos_theta + Chassic_Ch0_Real * sin_theta,
                        PID_Chassis.OUT_PID,
                        8000);
        break;

      case XTL_MODE:
      {
        f xtl_dir = (YK.yaogan.ch3 <= -600) ? 1.0f : -1.0f;
        DP.ML_Data_Deal(Chassic_Ch0_Real * cos_theta - Chassic_Ch1_Real * sin_theta,
                        Chassic_Ch1_Real * cos_theta + Chassic_Ch0_Real * sin_theta,
                        xtl_dir * XTL_SPEED_OUT,
                        8000); // 10000
        break;
      }

      case PLAYER_MODE:
        PLAYER_deal();
        break;

      default:
        DP.ML_Data_Deal(0, 0, 0, 0);
        break;
      }
  }
}
#ifndef PI
#define PI 3.1415926f
#endif
#define cater_RY 5
#define cater_RX 10
uint32_t name1[5] = {1, 2, 3, 4, 5};																								   // 4条准线，1个圆�?
graphic_tpyedef graphic_tpye1[7] = {Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line}; // 选择要绘制的图形		//30
Color_tpyedef color1[7] = {Color_Yellow, Color_Pink, Color_Yellow, Color_Green, Color_Yellow, Color_Yellow, Color_Red_Or_Bule};		   // 选择颜色
uint16_t d1[8][5] = {{0},
					 {0},
					 {1, 1, 1, 1, 1},
					 {0, 925, 925, 925, 955},
					 {0, 500, 490, 480, 540},
					 {0, 0, 0, 0, 0},
					 {0, 985, 985, 985, 955},
					 {0, 500, 490, 480, 435}};

float sin_theta_L, sin_theta_R, sin_theta_down, cos_theta_L, cos_theta_R, cos_theta_down;
uint32_t name2[7] = {6, 7, 8, 9, 10, 11, 12};
graphic_tpyedef graphic_tpye2[7] = {Graphic_Circle, Graphic_Line, Graphic_Line, Graphic_Rectangle, Graphic_Rectangle, Graphic_Rectangle, Graphic_Rectangle};
Color_tpyedef color2[7] = {Color_Green,
						   Color_Pink,
						   Color_White,
						   Color_Orange,
						   Color_Orange,
						   Color_Orange,
						   Color_Orange};

uint16_t d2[8][7] = {{0},
					 {0},
					 {2, 6, 0, 2, 2, 2, 2},
					 {480, 480, 700, 0, 0, 0, 0}, // 960x
					 {750, 750, 80, 0, 0, 0, 0},  // 540y
					 {75, 0, 0, 0, 0, 0, 0},
					 {0, 0, 0, 0, 0, 0, 0},
					 {0, 0, 80, 0, 0, 0, 0}};

uint32_t name4[5] = {15, 16, 17, 18};
graphic_tpyedef graphic_tpye4[5] = {Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line};
Color_tpyedef color4[5] = {Color_Yellow, Color_Yellow, Color_Yellow, Color_Yellow};

uint16_t d4[8][5] = {{0}, {0}, {2, 2, 2, 2}, {500, 800, 1470, 1170}, {20, 434, 20, 434}, {0, 0, 0, 0}, {800, 820, 1170, 1150}, {434, 434, 434, 434}};

//		uint16_t d4[8][5]={{0},{0},{2,2,2,2},{500,600,1470,1370},{20,158,20,158},{0,0,0,0},
//			                                     {600,720,1370,1250},{158,158,158,158}};

uint32_t key[5] = {21, 22, 23};
graphic_tpyedef key_1[5] = {Graphic_Rectangle, Graphic_Rectangle, Graphic_Rectangle, Graphic_Rectangle}; //,Color_White  ,Color_Red_Or_BuleColor_White
Color_tpyedef color_white_white_white[5] = {Color_White, Color_White, Color_White};
Color_tpyedef color_white_white_red[5] = {Color_White, Color_White, Color_Red_Or_Bule};
Color_tpyedef color_white_red_white[5] = {Color_White, Color_Red_Or_Bule, Color_White};
Color_tpyedef color_white_red_red[5] = {Color_White, Color_Red_Or_Bule, Color_Red_Or_Bule};
Color_tpyedef color_red_white_white[5] = {Color_Red_Or_Bule, Color_White, Color_White};
Color_tpyedef color_red_white_red[5] = {Color_Red_Or_Bule, Color_White, Color_Red_Or_Bule};
Color_tpyedef color_red_red_white[5] = {Color_Red_Or_Bule, Color_Red_Or_Bule, Color_White};
Color_tpyedef color_red_red_red[5] = {Color_Red_Or_Bule, Color_Red_Or_Bule, Color_Red_Or_Bule};
uint16_t d6[8][5]{{0}, {0}, {10, 10, 10}, {730, 830, 930}, {190, 190, 190}, {0, 0}, {800, 900, 1000}, {120, 120, 120}};

void App_Chassis_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3) // 1 kHz leg control
  {
    // TIM3 is 1 kHz for the leg loop. Keep chassis ramp/power work at 500 Hz.
    if (Motor_Flag.TIM3_Flag == 0)
    {
      KEY_Forback_Ctrl1();
      Chassic_Forback_Ctrl();
      M3508_Limit_deal();
      Motor_Flag.TIM3_Flag = 1;
      Leg_Update_Forward_State(0.002f);
    }
    else
    {
      Motor_Flag.TIM3_Flag = 0;
    }

    {
      LegControlInput leg_input = {0};
      const float leg_dt = 0.001f;

      float wheel_target_mean =
          (fabsf(DP.ML.qz) + fabsf(DP.ML.hz) +
           fabsf(DP.ML.hy) + fabsf(DP.ML.qy)) * 0.25f;
      float wheel_feedback_mean =
          (fabsf((float)M3508_MOTOR_QZ.sp) + fabsf((float)M3508_MOTOR_HZ.sp) +
           fabsf((float)M3508_MOTOR_HY.sp) + fabsf((float)M3508_MOTOR_QY.sp)) * 0.25f;
      float wheel_stall_ratio = 0.0f;
      if (wheel_target_mean > 100.0f)
        wheel_stall_ratio = LIMIT((wheel_target_mean - wheel_feedback_mean) /
                                      wheel_target_mean,
                                  0.0f, 1.0f);

      uint32_t now_tick = HAL_GetTick();
      uint32_t pitch_age_ms = now_tick - Gimbal_Pitch_Tick;
      uint32_t pitch_rate_age_ms = now_tick - Gimbal_Pitch_Acc_Tick;
      if (pitch_rate_age_ms > pitch_age_ms)
        pitch_age_ms = pitch_rate_age_ms;
      uint32_t roll_age_ms = now_tick - Gimbal_Roll_Tick;
      uint32_t roll_rate_age_ms = now_tick - Gimbal_Roll_Acc_Tick;
      if (roll_rate_age_ms > roll_age_ms)
        roll_age_ms = roll_rate_age_ms;

      leg_input.yk_mode = YK_Mode;
      leg_input.xtl_flag = XTL_Flag;
      leg_input.ch3 = YK.yaogan.v;
      leg_input.key_e = YK.Pressed_Check(KEY_PRESSED_E);
      leg_input.key_ctrl = YK.Pressed_Check(KEY_PRESSED_CTRL);
      leg_input.gimbal_roll = Gimbal_Pitch;
      leg_input.gimbal_pitch = Gimbal_Roll;
      leg_input.gimbal_roll_acc = Gimbal_Pitch_Acc;
      leg_input.gimbal_pitch_acc = Gimbal_Roll_Acc;
      leg_input.fb_real_speed = FB_Real_Speed;
      leg_input.lr_real_speed = LR_Real_Speed;
      leg_input.chassis_ch0_real = Chassic_Ch0_Real;
      leg_input.chassis_ch1_real = Chassic_Ch1_Real;
      leg_input.sin_theta = sin_theta;
      leg_input.cos_theta = cos_theta;
      leg_input.dt = leg_dt;
      leg_input.forward_cmd = Leg_Get_Forward_Command();
      leg_input.forward_accel = Leg_Forward_Accel;
      leg_input.wheel_stall_ratio = wheel_stall_ratio;
      leg_input.pitch_data_age = pitch_age_ms * 0.001f;
      leg_input.roll_data_age = roll_age_ms * 0.001f;
      leg_input.left_motor = &Left_Leg;
      leg_input.right_motor = &Right_Leg;
      //Leg_Control_Update(&leg_input);
      Leg_SMC_Control(&leg_input);
      LegControlOutput leg_control_output;
      Leg_Control_Get_Output(&leg_control_output);
      Leg_Trace_Update(&leg_input, &leg_control_output);
    }
    static uint16_t re_flag;
    re_flag++;
    if (re_flag % 2000 == 0)
    {
      if (Left_Leg.ERR != 1 &&
          HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 1)
        Left_Leg.DM_Start(0x02);
      if (Right_Leg.ERR != 1 &&
          HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 1)
        Right_Leg.DM_Start(0x01);
      re_flag = 0;
    }
    // Send the same control sample to both DM joints in this 1 ms period.
    // If either mailbox is unavailable, report a pair failure instead of
    // updating one joint with a stale counterpart.
    uint8_t dm_pair_ok = 0;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 2)
    {
      HAL_StatusTypeDef left_status =
          Left_Leg.DM_MIT(0x02, 0, 0, 0, Leg_Control_Get_Mit_Kd(),
                          Leg_Control_Get_Left_Torque());
      HAL_StatusTypeDef right_status =
          Right_Leg.DM_MIT(0x01, 0, 0, 0, Leg_Control_Get_Mit_Kd(),
                           Leg_Control_Get_Right_Torque());
      dm_pair_ok = (left_status == HAL_OK && right_status == HAL_OK);
    }
    Leg_Control_Report_OutputPairResult(dm_pair_ok);
  }
  if (htim == &htim5) // CAN通讯 100Hz
  {
    // 键盘发�?�到云台
    static uint8_t CNC_flag = 1;
    if (CNC_flag)
    {
      CAN_Communicate.Send_RM(0x558, Communicate_Send_Flag_1,
                              0,
                              CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_heat_limit,
                              CP.ext_power_heat_data_t.shooter_id1_17mm_cooling_heat);
      CNC_flag = 0;
    }
    else
    {
      CAN_Communicate.Send_RM(0x121, CP.ext_game_robot_status_t.chassis_power_limit,
                              1,
                              0,
                              0);
      CNC_flag = 1;
    }
  }

    if (htim == &htim2)
	{
		/*************************************************************************************************************************/

		// (uint16_t)(75 + 70 * Hode + (70 * Long_Hit_Hode));R=75
		sin_theta_L = sinf(theta_rad - PI / 2.0f);
		cos_theta_L = cosf(theta_rad - PI / 2.0f);
		sin_theta_down = sinf(theta_rad - PI);
		cos_theta_down = cosf(theta_rad - PI);
		sin_theta_R = sinf(theta_rad + PI / 2.0f);
		cos_theta_R = cosf(theta_rad + PI / 2.0f);

		d2[6][1] = (uint16_t)(410 + (1 + sin_theta) * 70);
		d2[7][1] = (uint16_t)(680 + (1 + cos_theta) * 70);

		d2[3][3] = (uint16_t)(410 - cater_RX + (1 - sin_theta) * 70);
		d2[4][3] = (uint16_t)(680 + cater_RY + (1 - cos_theta) * 70);
		d2[6][3] = (uint16_t)(410 + cater_RX + (1 - sin_theta) * 70);
		d2[7][3] = (uint16_t)(680 - cater_RY + (1 - cos_theta) * 70);

		d2[3][4] = (uint16_t)(410 - cater_RX + (1 - sin_theta_L) * 70);
		d2[4][4] = (uint16_t)(680 + cater_RY + (1 - cos_theta_L) * 70);
		d2[6][4] = (uint16_t)(410 + cater_RX + (1 - sin_theta_L) * 70);
		d2[7][4] = (uint16_t)(680 - cater_RY + (1 - cos_theta_L) * 70);

		d2[3][5] = (uint16_t)(410 - cater_RX + (1 - sin_theta_down) * 70);
		d2[4][5] = (uint16_t)(680 + cater_RY + (1 - cos_theta_down) * 70);
		d2[6][5] = (uint16_t)(410 + cater_RX + (1 - sin_theta_down) * 70);
		d2[7][5] = (uint16_t)(680 - cater_RY + (1 - cos_theta_down) * 70);

		d2[3][6] = (uint16_t)(410 - cater_RX + (1 - sin_theta_R) * 70);
		d2[4][6] = (uint16_t)(680 + cater_RY + (1 - cos_theta_R) * 70);
		d2[6][6] = (uint16_t)(410 + cater_RX + (1 - sin_theta_R) * 70);
		d2[7][6] = (uint16_t)(680 - cater_RY + (1 - cos_theta_R) * 70);
		/***********************************************************************************************************************************/

		static uint32_t ten_flag = 0;
		static uint8_t first_flag = 1;
		if (ten_flag > 600)
			ten_flag = 1;
		if (Q_Flag)
		{
			CP_DrawOrDelete_Char(35, Modify_Graphic, 0, (Color_tpyedef)Q_Char[0], Q_Char[1], Q_Char[2], Q_Char[3], Q_Char[4], (uint8_t *)"Q");
			Q_Flag++;
			if (Q_Flag > 3)
				Q_Flag = 0;
		}
		else if (E_Flag)
		{
			CP_DrawOrDelete_Char(36, Modify_Graphic, 0, (Color_tpyedef)E_Char[0], E_Char[1], E_Char[2], E_Char[3], E_Char[4], (uint8_t *)"V");
			E_Flag++;
			if (E_Flag > 3)
				E_Flag = 0;
		}
    else if(X_Flag)
    {
      CP_DrawOrDelete_Char(37, Modify_Graphic, 0, (Color_tpyedef)X_Char[0], X_Char[1], X_Char[2], X_Char[3], X_Char[4], (uint8_t *)"X");
      X_Flag++;
      if(X_Flag > 3)
        X_Flag = 0;
    }
    else if(MID_Flag)
    {
      CP_DrawOrDelete_Char(38, Modify_Graphic, 0, (Color_tpyedef)MID_Char[0], MID_Char[1], MID_Char[2], MID_Char[3], MID_Char[4], (uint8_t *)"MID");
      MID_Flag++;
      if(MID_Flag > 3)
        MID_Flag = 0;
    }
		else if (ten_flag % 30 == 0)
		{
			switch (UI_step_ten)
			{
			case 1:
				CP_DrawOrDelete_Seven_Graphic(name4, Increase_Graphic, graphic_tpye4, 1, color4, d4[0], d4[1], d4[2], d4[3], d4[4], d4[5], d4[6], d4[7]); // 边框
				UI_step_ten++;
				break;
			case 2:
				CP_DrawOrDelete_Five_Graphic(name1, Increase_Graphic, graphic_tpye1, 2, color1, d1[0], d1[1], d1[2], d1[3], d1[4], d1[5], d1[6], d1[7]); // 准星
				UI_step_ten++;
				break;
				//					case 3:
				//						CP_DrawOrDelete_Five_Graphic(key,Increase_Graphic,key_1,1,color_white_white_white,d6[0],d6[1],d6[2],d6[3],d6[4],d6[5],d6[6],d6[7]);//方框
				//						UI_step_ten++;
				//					break;
			case 3:
				CP_DrawOrDelete_Seven_Graphic(name2, Increase_Graphic, graphic_tpye2, 2, color2, d2[0], d2[1], d2[2], d2[3], d2[4], d2[5], d2[6], d2[7]);
				UI_step_ten++;
				break;
			case 4:
				CP_DrawOrDelete_One_Number(33, Increase_Graphic, Graphic_Int_number, 0, Color_Green, 25, 1, 5, 1150, 600, CP.ext_shoot_data_t.bullet_speed); // 底盘调参PID上限//MCL_SPEED
				UI_step_ten++;
				break;
			case 5:
				CP_DrawOrDelete_One_Number(34, Increase_Graphic, Graphic_Int_number, 0, Color_Green, 25, 1, 5, 1150, 550, Gimbal_Roll); // 底盘调参PID下限//MCL_SPEED
				UI_step_ten++;
				break;
			case 6:
				CP_DrawOrDelete_Char(35, Increase_Graphic, 0, (Color_tpyedef)Q_Char[0], Q_Char[1], Q_Char[2], Q_Char[3], Q_Char[4], (uint8_t *)"Q");
				UI_step_ten++;
				break;
			case 7:
				CP_DrawOrDelete_Char(36, Increase_Graphic, 0, (Color_tpyedef)E_Char[0], E_Char[1], E_Char[2], E_Char[3], E_Char[4], (uint8_t *)"V");
				UI_step_ten++;
				break;
			case 8:
				CP_DrawOrDelete_One_Number(32, Increase_Graphic, Graphic_Float_number, 0, Color_Green, 25, 1, 5, 1150, 500, V_Cap_Real); // 电容
				UI_step_ten++;
				break;
      case 9:
         CP_DrawOrDelete_Char(37, Increase_Graphic, 0, (Color_tpyedef)X_Char[0], X_Char[1], X_Char[2], X_Char[3], X_Char[4], (uint8_t *)"X");
         UI_step_ten++;
         break;
      case 10:
         CP_DrawOrDelete_Char(38, Increase_Graphic, 0, (Color_tpyedef)MID_Char[0], MID_Char[1], MID_Char[2], MID_Char[3], MID_Char[4], (uint8_t *)"MID");
         UI_step_ten =1;
         break;
			default:
				CP_DrawOrDelete_Five_Graphic(name1, Increase_Graphic, graphic_tpye1, 2, color1, d1[0], d1[1], d1[2], d1[3], d1[4], d1[5], d1[6], d1[7]); // 准星
				UI_step_ten = 1;
				break;

				//                CP_DrawOrDelete_Char(31, Increase_Graphic, 0, (Color_tpyedef)Q_Char[0], Q_Char[1], Q_Char[2], Q_Char[3], Q_Char[4], (uint8_t *)"Q");
			}
		}
		else
		{
			switch (UI_step)
			{
			case 1:
				CP_DrawOrDelete_Seven_Graphic(name2, Modify_Graphic, graphic_tpye2, 2, color2, d2[0], d2[1], d2[2], d2[3], d2[4], d2[5], d2[6], d2[7]);
				UI_step++;
				break;
			case 2:
				CP_DrawOrDelete_One_Number(32, Modify_Graphic, Graphic_Float_number, 0, Color_Green, 25, 1, 5, 1150, 500, V_Cap_Real); // 电容
				UI_step++;
				break;
			case 3:
				CP_DrawOrDelete_Seven_Graphic(name2, Modify_Graphic, graphic_tpye2, 2, color2, d2[0], d2[1], d2[2], d2[3], d2[4], d2[5], d2[6], d2[7]);
				UI_step++;
				break;
			case 4:
				static uint8_t count_g = 0;
				if (count_g % 2 == 0)
				{
					count_g = 1;
					CP_DrawOrDelete_One_Number(33, Modify_Graphic, Graphic_Int_number, 0, Color_Green, 25, 1, 5, 1150, 600, CP.ext_shoot_data_t.bullet_speed); // 底盘调参PID上限
				}
				else
				{
					count_g = 2;
					CP_DrawOrDelete_One_Number(34, Modify_Graphic, Graphic_Int_number, 0, Color_Green, 25, 1, 5, 1150, 550, Gimbal_Roll); // 底盘调参PID下限
				}
				UI_step++;
				break;
			case 5:
				CP_DrawOrDelete_Seven_Graphic(name2, Modify_Graphic, graphic_tpye2, 2, color2, d2[0], d2[1], d2[2], d2[3], d2[4], d2[5], d2[6], d2[7]);
				UI_step = 1;
				break;
			default:
				UI_step = 1;
				break;
			}
		}
		ten_flag++;
	}
}

void App_Chassis_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &CP_SYSTEM_USART_HANDLE)
  {
    CP_System_DMACplt_DataDeal();
  }
}
