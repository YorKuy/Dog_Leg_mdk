/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// 添加用户自定义的头文�??
#include "stdint.h"
#include "RM_Lib.h"
#include "CP_System.h"
#include "DM.h"
#include "../../Control_Lib/leg.h"
#include "packet.h"
#include "imu_data_decode.h"
#include <math.h>
#include "hipnuc_dec.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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

#define Motor_Yaw_front -1.46722f // 弧度
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
void SystemClock_Config(void);
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

  if (FB_Real_Speed > 0)
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
    if (CAN_Communicate.rx_buf[0] == 0)
      Gimbal_Roll = angle.f;
    else if (CAN_Communicate.rx_buf[0] == 1)
      Gimbal_Pitch = angle.f;
    else if (CAN_Communicate.rx_buf[0] == 2)
      Gimbal_Roll_Acc = angle.f;
    else if (CAN_Communicate.rx_buf[0] == 3)
      Gimbal_Pitch_Acc = angle.f;
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

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    MODE_DEAL();
    YAW_Error = 0;
    Key_deal();
    cp_state = Judge_IF_DUM_Normal();
    // INFO("%.2f,%.2f,%.2f\n",gyr_x,gyr_y,gyr_z);
    // INFO("%.2f,%.2f,%.2f\n",Gimbal_Pitch,Right_Leg.mang*rad_T,Left_Leg.mang*rad_T);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
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

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3) // 2k
  {
    KEY_Forback_Ctrl1();
    Chassic_Forback_Ctrl();
    static uint16_t re_flag;
    if (Motor_Flag.TIM3_Flag == 0)
    {
      M3508_Limit_deal();
      Motor_Flag.TIM3_Flag = 1;
    }
    else
    {
      LegControlInput leg_input = {0};
      leg_input.yk_mode = YK_Mode;
      leg_input.xtl_flag = XTL_Flag;
      leg_input.ch3 = YK.yaogan.v;
      leg_input.key_e = YK.Pressed_Check(KEY_PRESSED_E);
      leg_input.key_ctrl = YK.Pressed_Check(KEY_PRESSED_CTRL);
      leg_input.gimbal_roll = Gimbal_Roll;
      leg_input.gimbal_pitch = Gimbal_Pitch;
      leg_input.gimbal_roll_acc = Gimbal_Roll_Acc;
      leg_input.gimbal_pitch_acc = Gimbal_Pitch_Acc;
      leg_input.fb_real_speed = FB_Real_Speed;
      leg_input.lr_real_speed = LR_Real_Speed;
      leg_input.chassis_ch0_real = Chassic_Ch0_Real;
      leg_input.chassis_ch1_real = Chassic_Ch1_Real;
      leg_input.sin_theta = sin_theta;
      leg_input.cos_theta = cos_theta;
      leg_input.left_motor = &Left_Leg;
      leg_input.right_motor = &Right_Leg;
      //Leg_Control_Update(&leg_input);
      Leg_SMC_Control(&leg_input);
      if (Motor_Flag.DM_Flag == 0)
      {
        Left_Leg.DM_MIT(0x02, 0, 0, 0, Leg_Control_Get_Mit_Kd(), Leg_Control_Get_Left_Torque());
        Motor_Flag.DM_Flag = 1;
      }
      else if (Motor_Flag.DM_Flag == 1)
      {
        Right_Leg.DM_MIT(0x01, 0, 0, 0, Leg_Control_Get_Mit_Kd(), Leg_Control_Get_Right_Torque());
        Motor_Flag.DM_Flag = 0;
      }
      Motor_Flag.TIM3_Flag = 0;
    }
    re_flag++;
    if (re_flag % 2000 == 0)
    {
      if (Motor_Flag.DM_Flag == 1 && Left_Leg.ERR != 1)
        Left_Leg.DM_Start(0x02);
      else if (Motor_Flag.DM_Flag == 0 && Right_Leg.ERR != 1)
        Right_Leg.DM_Start(0x01);
      re_flag = 0;
    }
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &CP_SYSTEM_USART_HANDLE)
  {
    CP_System_DMACplt_DataDeal();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
