#ifndef __RM_LIB_H
#define __RM_LIB_H

#include "main.h"
#include "stdio.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "string.h"
#include "math.h"
#include "can.h"			// 这行报错请屏蔽
#include "usart.h"		// 这行报错请屏蔽
#include "spi.h"			// 这行报错请屏蔽
#include "tim.h"			// 这行报错请屏蔽
#include "my_math.h"

/**************************************** USART **********************************************************/
#define USART_BUF_SIZE 		  128		// 数组大小，可修改
#define PRINTF_USART_HANDLE   huart2	// 串口号，可修改
extern uint8_t info_ubuf[USART_BUF_SIZE];
#define INFO(...) HAL_UART_Transmit (&PRINTF_USART_HANDLE,\
									(uint8_t *)info_ubuf,\
									sprintf((char *)info_ubuf,__VA_ARGS__),\
									0xffff)
									
/**************************************** C A N **********************************************************/
class USER_CAN
{
	public:
		CAN_HandleTypeDef* 	    hcan;
		CAN_TxHeaderTypeDef     TxHeader;
		CAN_RxHeaderTypeDef     RxHeader;
		uint8_t                 rx_buf[8];
		uint8_t                 tx_buf[8];
		bool					FIFO;
	
		void 					Init(uint16_t t,uint16_t x);
		HAL_StatusTypeDef		Send(uint16_t Id,uint8_t* pData);
		HAL_StatusTypeDef   Send_RM(uint16_t Id,int16_t M_201, int16_t M_202, int16_t M_203,int16_t M_204);
		HAL_StatusTypeDef 	Receive(CAN_HandleTypeDef *hcan);
	
	  HAL_StatusTypeDef   RMD_Read_and_Write_Things	(uint16_t Id,uint16_t Order_Id);
	  HAL_StatusTypeDef   RMD_Write_PID(uint16_t Id,uint16_t Order_Id,uint16_t anglePidKp,uint16_t anglePidKi,
	                                    uint16_t speedPidKp,uint16_t speedPidKi,uint16_t iqPidKp,uint16_t iqPidKi);
	  HAL_StatusTypeDef   RMD_Write_ACCLE_to_RAM(uint16_t Id,int32_t Accel);
	  HAL_StatusTypeDef   RMD_Write_EncoderOffset_to_ROM(uint16_t Id,uint16_t EncoderOffset);//写入编码器值
  	HAL_StatusTypeDef   RMD_Iqcontrol_Motor(uint16_t Id,int16_t iqControl);
	  HAL_StatusTypeDef   RMD_Speedcontrol_Motor(uint16_t Id,int32_t angleControl);
	  HAL_StatusTypeDef   RMD_Speedcontrol_Motor(uint16_t Id,uint16_t Order_Id,uint16_t maxSpeed,int32_t angleControl);
		HAL_StatusTypeDef   RMD_Anglecontrol_Motor(uint16_t Id,uint16_t Order_Id,uint8_t spinDirection,uint16_t maxSpeed,uint16_t angleControl);	
	  HAL_StatusTypeDef 	RMD_Receive(CAN_HandleTypeDef *hcan);
		
		USER_CAN(CAN_HandleTypeDef* p,bool fifo):hcan(p),FIFO(fifo){}
	private:
		uint32_t                TxMailbox;
		uint8_t 			    FreeTxNum;

};
class MOTOR_RM{
	public:
		const 		uint16_t ID;	// 电机反馈ID
		USER_CAN* 	can_rev;
	
		int16_t  	mang;      		// (int16_t) ((Can1_Data[0] << 8) | Can1_Data[1]);
		int16_t  	sp;        		// (int16_t) ((Can1_Data[2] << 8) | Can1_Data[3]);
		int16_t  	AT_current;     // (int16_t) ((Can1_Data[4] << 8) | Can1_Data[5]);
		int8_t   	temp;      		//  Can1_Data[6];
		int 		mang_inf;
		int 		motor_number;
	
		HAL_StatusTypeDef 		update(void);
		void 					update_mang_inf(void);
		MOTOR_RM(const uint16_t id,class USER_CAN* CAN_rev):ID(id),can_rev(CAN_rev){}
	private:
		uint8_t 	first;
		int 		Last_mang;
};

typedef struct{

	uint16_t anglePidKp;
	uint16_t anglePidKi;
	uint16_t speedPidKp;
	uint16_t speedPidKi;
  uint16_t iqPidKp;
	uint16_t iqPidKi;	
	int32_t  Accel;
	uint16_t encoder;          //编码器位置     0~65535
	uint16_t encoderRaw;       //编码器原始位置 0~65535
	uint16_t encoderOffset;	   //编码器零偏     0~65535
	int64_t motorAngle;
  uint16_t circleAngle;		
	int8_t temperature;
	uint16_t voltage;
	uint8_t eerorState;
  int16_t iq;
	int16_t speed;
	uint16_t now_encoder;  //编码器位置值	
	int16_t iA;
	int16_t iB;
	int16_t iC;	
	int16_t iqControl;		
  int32_t speedControl;
	int32_t angleControl;
		
}RMD_typedef;

class MOTOR_RMD{
	public:
		const uint16_t ID;	// 电机反馈ID
		USER_CAN* 	can_rev;
	  RMD_typedef  RMD_X;
	
    HAL_StatusTypeDef   RMD_update(void);
		MOTOR_RMD(const uint16_t id,class USER_CAN* CAN_rev):ID(id),can_rev(CAN_rev){}
		private:
			
};

typedef struct{
	float qy;
	float hy;
	float qz;
	float hz;
}ML_typedef;

class MOTOR_DiPan{
	public:
		ML_typedef 	ML;
	
		MOTOR_DiPan(void);
		void ML_Data_Deal(float lx,float ly,float lp,int MAX_rate);

};

/*************************************************** Duo Lun  ******************************************************/
#define M_PI       3.1415926535897932384626433832795f
#define COS_45     0.70710678118654752440084436210485f
#define RAD2MANG   1303.7972938088065906186957895476f    //8192/3.1415926/2

struct wheel_dir_and_weight
{
  bool dir;
  float speed;
  int yaogan_speed;
};
struct lun_xy{
  float x,y;
};

enum {
  QZ = 0,
  HZ,
  HY,
  QY
};

class RUDDER_DiPan
{
	public:
		int16_t g_angle_6020[4] = {0, 0,0,0};
    wheel_dir_and_weight g_wheel_3508[4] = {0,0,0,0};
		lun_xy QZ_xy,HZ_xy,HY_xy,QY_xy;
		uint16_t ZERO_0[4]  = {4758,2060,3415,7417};//qz,hz,hy,qy  每个舵轮都不一样，自己修改 6100,6200,2020,6170 正常 6156,6101,2093,6087 好一点//6141,6157,2020,6147 很好但是不正常的1991
		uint16_t ZERO_45[4] = {5782,3087,4400,249};//qz,hz,hy,qy  每个舵轮都不一样，自己修改 //7150
		#define Yaw_mid_of_0   4100
		#define Yaw_mid_of_45  3076 
		uint16_t follow_chassic=Yaw_mid_of_0;
	  void  Not_Xiaotuoluo_Jie_Suan(float CH0, float CH1, int16_t CH2,int16_t Angle_L,int16_t SPEED_MAX);
		void 	Xiaotuoluo_jie_Suan(uint16_t Mang_yaw,int16_t CH0,int16_t CH1,int16_t CH2,int16_t Angle,int16_t SPEED_MAX);
		
	private:	
		
		float Get_Ch0_Ch1_Vector_Speed(int16_t CH0, int16_t CH1);
	  float  Get_Max_Speed(int16_t CH0, int16_t CH1, int16_t CH2);
	  float absf(float d0);
  	float Get_Max_float(float d0,float d1);
	  int16_t Get_Max_int16(int16_t d0,int16_t d1);
	
};
/**************************************** P I D **********************************************************/

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);
#define LIMIT(x,min,max) ((x)<(min)?(min):((x)>(max)?(max):(x)))
#define NL   -3
#define NM	 -2
#define NS	 -1
#define ZE	 0
#define PS	 1
#define PM	 2
#define PL	 3

#define SF_LENGTH  5

class PID_class{
	public:
		float		error;
		float 		KP,KI,KD;
		float		LIMIT_P,LIMIT_I,LIMIT_D,LIMIT_PID,Integral;
		float 		LP_out_last ;
		float 		Deadzoom,Separate;
		float 		OUT_P,OUT_I,OUT_D,last_OUT_D,OUT_PID,delta_OUT_PID;
	  float 		g_speed_3508[4] = {0, 0, 0, 0};
		int16_t 	g_angle_6020[4] = {0, 0, 0, 0};
	
	  bool 		Get_6020mang_need_turn_direction_is(uint16_t goal, uint16_t now);
	  void 		PID_update_for_6020mang(int16_t goal, uint16_t now, struct wheel_dir_and_weight *wheel_dir_and_weight);
		void 		PID_new_update(float goal,float now);
		void 		PID_update(float goal,float now);
		void    PID_Inc_update(float goal,float now);
		void 	PID_update_LP(float goal, float now, float k_value);
		float 	low_pass_filter(float value, float k_value);
		PID_class(float kp,float ki,float kd,float limit_p,float limit_i,float limit_d,float limit_pid,float dz = 0,float separate = 0):
		KP(kp),KI(ki),KD(kd),LIMIT_P(limit_p),LIMIT_I(limit_i),LIMIT_D(limit_d),LIMIT_PID(limit_pid),Deadzoom(dz),Separate(separate){}
	private:
		float alpha=0.5;
		float LAST_Error;
		float last_error;
		float previous_error;
};

class PID_Fuzzy_class{
	public:
		float LastError;		//前次误差
		float error;			//当前误差
		float SumError;			//积分误差
		float IMax;					//积分限制
		float POut,IOut,DOut;	//比例输出
	  float DOut_last;    //上一次微分输出
		float OutMax;       //限幅
	  float Out;          //总输出
		float Out_last;     //上一次输出
		
		float I_U;          //变速积分上限
		float I_L;          //变速积分下限
	
	  float Kp0,Ki0,Kd0;          //PID初值
	  float dKp,dKi,dKd;          //PID变化量
	
    float stair,Kp_stair,Ki_stair,Kd_stair;	      //动态调整梯度   //0.25f
		
		void  FuzzyPID_update(float goal ,float now);
		PID_Fuzzy_class(float kp,float ki,float kd,float limit_i,float limit_pid,float IL,float Stair,float KP_stair,float KI_stair,float KD_stair):
		Kp0(kp),Ki0(ki),Kd0(kd),IMax(limit_i),OutMax(limit_pid),I_L(IL),stair(Stair),Kp_stair(KP_stair),Ki_stair(KI_stair),Kd_stair(KD_stair){}		
		
	private:
		
	const float fuzzyRuleKp[7][7]={
		PL,	PL,	PM,	PM,	PS,	ZE,	ZE,
		PL,	PL,	PM,	PS,	PS,	ZE,	NS,
		PM,	PM,	PM,	PS,	ZE,	NS,	NS,
		PM,	PM,	PS,	ZE,	NS,	NM,	NM,
		PS,	PS,	ZE,	NS,	NS,	NM,	NM,
		PS,	ZE,	NS,	NM,	NM,	NM,	NL,
		ZE,	ZE,	NM,	NM,	NM,	NL,	NL
	};
	 
	const float fuzzyRuleKi[7][7]={
		NL,	NL,	NM,	NM,	NS,	ZE,	ZE,
		NL,	NL,	NM,	NS,	NS,	ZE,	ZE,
		NL,	NM,	NS,	NS,	ZE,	PS,	PS,
		NM,	NM,	NS,	ZE,	PS,	PM,	PM,
		NS,	NS,	ZE,	PS,	PS,	PM,	PL,
		ZE,	ZE,	PS,	PS,	PM,	PL,	PL,
		ZE,	ZE,	PS,	PM,	PM,	PL,	PL
	};
	 
	const float fuzzyRuleKd[7][7]={
		PS,	NS,	NL,	NL,	NL,	NM,	PS,
		PS,	NS,	NL,	NM,	NM,	NS,	ZE,
		ZE,	NS,	NM,	NM,	NS,	NS,	ZE,
		ZE,	NS,	NS,	NS,	NS,	NS,	ZE,
		ZE,	ZE,	ZE,	ZE,	ZE,	ZE,	ZE,
		PL,	NS,	PS,	PS,	PS,	PS,	PL,
		PL,	PM,	PM,	PM,	PS,	PS,	PL
	};
	
	void 	fuzzy(float goal,float now);
};

/****************************************** D B U S **********************************************************/
#define YK_SW_UP             	((uint16_t)1)
#define YK_SW_MID             ((uint16_t)3)
#define YK_SW_DOWN            ((uint16_t)2)

#define KEY_PRESSED_W 			((uint16_t)0x01<<0)
#define KEY_PRESSED_S 			((uint16_t)0x01<<1)
#define KEY_PRESSED_A 			((uint16_t)0x01<<2)
#define KEY_PRESSED_D 			((uint16_t)0x01<<3)
#define KEY_PRESSED_SHIFT 	((uint16_t)0x01<<4)
#define KEY_PRESSED_CTRL 		((uint16_t)0x01<<5)
#define KEY_PRESSED_Q 			((uint16_t)0x01<<6)
#define KEY_PRESSED_E 			((uint16_t)0x01<<7)
#define KEY_PRESSED_R 			((uint16_t)0x01<<8)
#define KEY_PRESSED_F 			((uint16_t)0x01<<9)
#define KEY_PRESSED_G 			((uint16_t)0x01<<10)
#define KEY_PRESSED_Z 			((uint16_t)0x01<<11)
#define KEY_PRESSED_X 			((uint16_t)0x01<<12)
#define KEY_PRESSED_C 			((uint16_t)0x01<<13)
#define KEY_PRESSED_V 			((uint16_t)0x01<<14)
#define KEY_PRESSED_B 			((uint16_t)0x01<<15)



typedef struct{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t v;
	uint8_t s1;
	uint8_t s2;
}DR16_yaogan_typedef;
typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;
}DR16_shubiao_typedef;

class	DBUS
{
	public:
		UART_HandleTypeDef *	huart;
		uint8_t 				dbus_rx_buffer[20];
		
		DR16_yaogan_typedef 	yaogan;
		DR16_shubiao_typedef 	shubiao;
		uint16_t 				jianpan;
		
	
		void feed_watchdog(void)
		{
			time_100ms=0;
		}
		HAL_StatusTypeDef receive_run(void)
		{
			return HAL_UART_Receive_IT(this->huart,this->dbus_rx_buffer,20);
		}
		HAL_StatusTypeDef receive_refresh(void)
		{
			return HAL_UART_AbortReceive_IT(this->huart);
		}
		
		void 					Init(void);
		void					DBUS_RxCplt_IRQHandler(void);
		void          jianpan_deal(void);
		void 					set_zero(void);
		void 					data_deal(void);
		void 					watchdog_run(void);
		void					can_receive_data_deal(uint8_t *buf);
		void 					V_can_receive_data_deal(uint8_t *buf);
		
		uint8_t 				Pressed_Check(uint16_t key_value);	//按下状态 返回1

		DBUS(UART_HandleTypeDef *p):huart(p){}
	
	private:
		uint8_t 	first;
		uint16_t	time_100ms;
		uint8_t		index;
		uint16_t 	delaycount[18];
		uint16_t 	last_jianpan;
	
		HAL_StatusTypeDef 		check_and_deal(void);
};


/**************************************** ADXRS290 **********************************************************/
#ifdef __SPI_H__

#define ADXRS290_ADI_ID 		0x00
#define ADXRS290_MEMS_ID  		0x01
#define ADXRS290_DEV_ID 		0x02
#define ADXRS290_REV_ID  		0x03
#define ADXRS290_SN0  			0x04
#define ADXRS290_SN1 			0x05
#define ADXRS290_SN2 			0x06
#define ADXRS290_SN3 			0x07
#define ADXRS290_DATAX0 		0x08
#define ADXRS290_DATAX1 		0x09
#define ADXRS290_DATAY0 		0x0A
#define ADXRS290_DATAY1 		0x0B
#define ADXRS290_TEMP0 			0x0C
#define ADXRS290_TEMP1 			0x0D
#define ADXRS290_POWER_CTL 		0x10
#define ADXRS290_Filter 		0x11
#define ADXRS290_DATA_READY 	0x12

typedef struct gyro{
	float v;
	float v_nonoise;
	float theta_euler;
	float bias;
	uint32_t dev_count;
}ADXRS290_TYPEDEF;

typedef enum 
{
	ADXRS290_OK       		= 0x00U,
	ADXRS290_SET_ERROR    	= 0x01U,
	ADXRS290_ID_ERROR    	= 0x02U,
	ADXRS290_ERROR    		= 0x03U,
} ADXRS290_StatusTypeDef;


class ADXRS290
{
	public:
		SPI_HandleTypeDef *		hspi;
		GPIO_TypeDef *				GPIOx;
		uint16_t 							GPIO_Pin;

		ADXRS290_TYPEDEF		sensor_data_X;
		ADXRS290_TYPEDEF		sensor_data_Y;
	
		ADXRS290_StatusTypeDef		Init(uint8_t hpf_corner,uint8_t odr_lpf);
		void											adxrs290_update(void);
		ADXRS290_StatusTypeDef		adxrs290_writeByte(uint8_t subAddress, uint8_t data);
		uint8_t										adxrs290_readByte(uint8_t subAddress);
		void 											adxrs290_readBytes(uint8_t subAddress, uint8_t count, uint8_t* spi_rev_buf);
		
		ADXRS290(SPI_HandleTypeDef *q,GPIO_TypeDef *w,uint16_t e,uint16_t t,float y,const char *u):
		hspi(q),GPIOx(w),GPIO_Pin(e),SELF_TEST_NUM_290(t),DEAD_ZONE_290(y),string_check_290(u){}
		private:
		uint16_t 		SELF_TEST_NUM_290;
		float 			DEAD_ZONE_290;
		const char*		string_check_290;
		void ADXRS290_SPI_ON()
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
		}
		void ADXRS290_SPI_OFF()
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
		}

};
#endif


/*****************************************ADXRS453*******************************************/
#ifdef __SPI_H__


typedef struct {
	float v;
	float v_nonoise;
	float theta_euler;
	float bias;
	float offset_v;
	float offset_max;
	float offset_min;
	uint32_t dev_count;
	uint8_t calibration;
}ADXRS453_TYPEDEF;
typedef enum 
{
	ADXRS453_OK       		= 0x00U,
	ADXRS453_RW_ERROR   	= 0x01U,
	ADXRS453_P0_ERROR   	= 0x02U,
	ADXRS453_P1_ERROR  		= 0x03U,
	ADXRS453_SPI_ERROR		= 0x04U,
	ADXRS453_RE_ERROR		= 0x05U,
	ADXRS453_DU_ERROR		= 0x06U,
	ADXRS453_PLL_ERROR		= 0x07U,
	ADXRS453_Q_ERROR		= 0x08U,
	ADXRS453_NVM_ERROR		= 0x09U,
	ADXRS453_POR_ERROR		= 0x0AU,
	ADXRS453_PWR_ERROR		= 0x0BU,
	ADXRS453_CST_ERROR		= 0x0CU,
	ADXRS453_CHK_ERROR		= 0x0DU,
	ADXRS453_ERROR
} ADXRS453_StatusTypeDef;

class ADXRS453
{
	public:
		SPI_HandleTypeDef *		hspi;
		GPIO_TypeDef *			GPIOx;
		uint16_t 				GPIO_Pin;
		TIM_HandleTypeDef *		htim;
		
		ADXRS453_TYPEDEF 		sensor_data;
		
		ADXRS453_StatusTypeDef	Init();
		ADXRS453_StatusTypeDef  adxrs453_update(void);
		ADXRS453_StatusTypeDef	sensor(bool CHK,int16_t* date);
		uint32_t 								TransmitReceive(uint32_t address);
		ADXRS453_StatusTypeDef 	addread(uint8_t address,int16_t* date);
	
		ADXRS453(SPI_HandleTypeDef *q,GPIO_TypeDef *w,uint16_t e,TIM_HandleTypeDef *r,uint16_t t,float y,const char *u):
		hspi(q),GPIOx(w),GPIO_Pin(e),htim(r),SELF_TEST_NUM(t),DEAD_ZONE(y),string_check(u){}
	
	private:
		uint16_t 		SELF_TEST_NUM;
		float 			DEAD_ZONE;
		const char	*	string_check;
		void SPI_ON()
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
		}
		void SPI_OFF()
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
		}
		bool 			odd_check(uint32_t date);
};
#endif


/**************************************BMI088************************************************/
#ifdef __SPI_H__

#define  ACC_CHIP_ID  		0x00
#define  ACC_ERR_REG			0X02
#define  ACC_STATUS				0X03
#define  ACC_X_LSB				0X12
#define  ACC_X_MSB				0X13
#define  ACC_Y_LSB				0X14
#define  ACC_Y_MSB				0X15
#define  ACC_Z_LSB				0X16
#define  ACC_Z_MSB				0X17
#define  SENSORTIME_0			0X18
#define  SENSORTIME_1			0X19
#define  SENSORTIME_2			0X1A
#define  ACC_INT_STAT_1		0X1D
#define  TEMP_MSB					0X22
#define  TEMP_LSB					0X23
#define  ACC_CONF					0X40
#define  ACC_RANGE				0X41
#define  INT1_IO_CTRL			0X53
#define  INT2_IO_CTRL			0X54
#define  INT_MAP_DATA			0X58
#define  ACC_SELF_TEST		0X6D
#define  ACC_PWR_CONF			0X7C
#define  ACC_PWR_CTRL			0X7D
#define  ACC_SOFTRESET		0X7E

#define  GYRO_CHIP_ID				0X00
#define  RATE_X_LSB					0X02
#define  RATE_X_MSB					0X03
#define  RATE_Y_LSB					0X04
#define  RATE_Y_MSB					0X05
#define  RATE_Z_LSB					0X06
#define  RATE_Z_MSB					0X07
#define  GYRO_INT_STAT_1		0X0A
#define  GYRO_RANGE					0X0F
#define  GYRO_BANDWIDTH			0X10
#define  GYRO_LPM1					0X11
#define  GYRO_SOFTRESET			0X14
#define  GYRO_INT_CTRL			0X15
#define  INT3_INT4_IO_CONF	0X16
#define  INT3_INT4_IO_MAP		0X18
#define  GYRO_SELF_TEST			0X3C
float inVSqrt(float x);
typedef struct {
	struct {
		float x;
		float y;
		float z;	
		float LPF_x;
		float LPF_y;
		float LPF_z;		
	} acc;
	
	struct {
		float x_nonoise;
		float y_nonoise;
		float z_nonoise;	
	struct {
			float x;
			float y;
			float z;
		} calibration;	
	struct {
			float x;
			float y;
			float z;
		} origin;			
	struct {
		float x;
		float y;
		float z;
	} dynamicSum;
	struct {
		float x;
		float y;
		float z;
	} offset;	
	struct {
		float x;
		float y;
		float z;
	} offset_max;	
	struct {
		float x;
		float y;
		float z;
	} offset_min;	
	struct {
		float x;
		float y;
		float z;
	} dps;		
	struct {
		float x;
		float y;
		float z;
	} LPF;			
	} gyro;
	
	struct {
		float x;
		float y;
		float z;
		float now_x;
		float now_y;
		float now_z;
		float last_x;
		float last_y;
		float last_z;	
		float Real_x;
		float Real_y;
		float Real_z;			
	} mang;
	
	uint16_t runningTimes;
	float temperature;
	uint8_t calibration;
	
}BMI088_TYPEDEF;

typedef enum 
{
	BMI088_OK       		  = 0x00U,
	BMI088_SET_ERROR      = 0x01U,
	BMI088_ACC_ID_ERROR	  = 0x02U,
	BMI088_GYRO_ID_ERROR  = 0x03U,
	BMI088_ERROR    		  = 0x04U,
	BMI088_SELFTEXT_ERROR = 0x05U,
} BMI088_StatusTypeDef;

typedef enum 
{
	BMI088_GYRO_RANGE_2000 = 0x00U,
	BMI088_GYRO_RANGE_1000 = 0x01U,
	BMI088_GYRO_RANGE_500  = 0x02U,
	BMI088_GYRO_RANGE_250  = 0x03U,
	BMI088_GYRO_RANGE_125  = 0x04U,
} BMI088_GyroRangeTypeDef;

typedef enum 
{
	BMI088_ACC_RANGE_3  =  0X00U,
	BMI088_ACC_RANGE_6  =  0X01U,
	BMI088_ACC_RANGE_12 =  0X02U,
	BMI088_ACC_RANGE_24 =  0X03U,
}BMI088_AccRangeTypeDef;

struct{
	float CUTOFF_FREQ = 500.0f;     //截止频率
	float SAMPLE_RATE = 0.001f;    //采样周期
	float pi = 3.1415926; //π
	float alpha;     //滤波系数
}LPF_factor;



typedef struct{
    int16_t roundYaw;
    int16_t roundPitch;
    int16_t roundRoll;
} angleRound_t;

/*四元数↓*/


typedef struct{
    int16_t roundYaw;
    int16_t roundPitch;
    int16_t roundRoll;
} angleRound;

/*二维float向量结构体*/

void BMI_CrossRound_err(void);

typedef struct vec2f
{
	float data[2];
} vec2f;

/*二维int16向量结构体*/

typedef struct vec2int16
{
	 short data[2];
} vec2int16;

/*三维float向量结构体*/

typedef struct vec3f
{
	float data[3];
} vec3f;

/*三维int16向量结构体*/

typedef struct vec3int16
{
	 short data[3];
} vec3int16;

/*四维float向量结构体*/

typedef struct vec4f
{
	float data[4];
}vec4f;

/*四维int16向量结构体*/

typedef struct vec4int16
{
	 short data[4];
}vec4int16;


/*结构体*/

typedef struct accdata
{
    vec3int16 origin;  //原始值
    vec3f offset_max;  //零偏值最大值
    vec3f offset_min;  //零偏值最小值	
    vec3f offset;      //零偏值 
    vec3f calibration; //校准值
    vec3f filter;      //滑动平均滤波值
	vec3f accValue;	   //加速度值，单位：m/s2
	vec3f dynamicSum;  //校准时求和计算
	uint16_t runningTimes;//运行次数
} accdata;

typedef struct gyrodata
{
    vec3int16 origin;  //原始值
    vec3f offset_max;  //零偏值最大值
    vec3f offset_min;  //零偏值最小值	
    vec3f offset;      //零偏值 
    vec3f calibration; //校准值
    vec3f filter;      //滑动平均滤波值
    vec3f dps;         //度每秒 
    vec3f radps;       //弧度每秒
    vec3f dynamicSum;  //校准时求和计算
} gyrodata;

typedef 	struct {
		float x;
		float y;
		float z;	
	} Deal_acc_t;
typedef 	struct {
		float x;
		float y;
		float z;	
	} Deal_gyro_t;
	
typedef struct{
	float q0;
	float q1;
	float q2;
	float q3;
}quaterInfo_t;

typedef struct{
	float pitch;
	float roll;
	float yaw;
}eulerianAngles_t;

typedef struct{
	float pitch;
	float roll;
	float yaw;
	float Deal_pitch;
	float Deal_roll;
	float Deal_yaw;	
}Anglespeed_t;


/*四元数↑*/

class BMI088
{
	public:
		SPI_HandleTypeDef     *hspi;
		TIM_HandleTypeDef 		*htim;
		GPIO_TypeDef 					*CSB1_GPIOx,  *CSB2_GPIOx;
		uint16_t 							CSB1_GPIO_Pin,CSB2_GPIO_Pin;
		angleRound_t       	Round;
		BMI088_TYPEDEF		sensor_data;
		BMI088_StatusTypeDef	Init_High(void);		
		BMI088_StatusTypeDef	Init(void);
		Deal_acc_t Deal_acc;
		Deal_gyro_t Deal_gyro;
		accdata acc;
		gyrodata gyro;
	  eulerianAngles_t eulerAngle;  	//欧拉角
		eulerianAngles_t lastAngle;			//上一次的欧拉角
		eulerianAngles_t nowAngle;			//现在的欧拉角
		eulerianAngles_t realAngle;			//现在真实的欧拉角（已经叠加了圈数）	
		Anglespeed_t Anglespeed;
    float q0_t;
    float q1_t;
    float q2_t;
    float q3_t;
	float out_last;
	float out;
		quaterInfo_t Q_info = {1,0,0,0}; 	//全局四元数
		void									BMI088_write_Acc(uint8_t subAddress,uint8_t data);
		void									BMI088_write_Gyro(uint8_t subAddress,uint8_t data);
		void 									BMI088_read_Acc(uint8_t subAddress, uint8_t len, uint8_t* spi_rev_buf);
		void 									BMI088_read_Gyro(uint8_t subAddress, uint8_t len, uint8_t* spi_rev_buf);
		void 									set_zero(void);
		void 									low_pass_filter_init(void);
		float 									low_pass_filter(float value);
		void    								BMI088_update(void);
		void									BMI088_New_update(void);
		void									BMI_Get_EulerAngle(void);
		void 									getValues(void);
		void 									QuatToEulerAngles(void);
		void 									analyse(void);
		void 									BMI_analyse(void);	
		void 									BMI_QuatToEulerAngles(void);
		void 									BMI_CrossRound(void);
		void 									BMI_CrossRound_err(void);
		void 									Analyse_speed(void);
		void 									BMI088_AHRS(float gx, float gy, float gz, float ax, float ay, float az);
		BMI088(SPI_HandleTypeDef *q,TIM_HandleTypeDef *t,GPIO_TypeDef *w1,uint16_t p1,GPIO_TypeDef *w2,uint16_t p2,uint16_t num,float dz,BMI088_GyroRangeTypeDef gyrorange,BMI088_AccRangeTypeDef accrange):
		hspi(q),htim(t),CSB1_GPIOx(w1),CSB1_GPIO_Pin(p1),CSB2_GPIOx(w2),CSB2_GPIO_Pin(p2),SELF_TEST_NUM(num),dead_zoom(dz),GyroRange(gyrorange),AccRange(accrange){}
	private:
		BMI088_GyroRangeTypeDef  GyroRange;  // 陀螺仪量程
  	BMI088_AccRangeTypeDef   AccRange;  // 加速度量程
		float 				GyroResolution;   // 陀螺仪分辨率
	  float 				AccRangsetting;   // 设置量程为
		float         Acc_Temperature_Offset=0,Gyro_Temperature_Offset=0;
		float         dead_zoom;
		uint8_t				enable_acc;
		uint16_t 			SELF_TEST_NUM;
		const char*		string_check_088;
		uint16_t      timer_1ms=0;
		uint8_t       selftext_error_flag=0,selftext_reset_step=0;
		float 				last_gyro_x,last_gyro_y,last_gyro_z,last_temperature,filter_count_x,filter_count_y,filter_count_z,filter_count_temperature;
	
		void BMI088_SPI_ON(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
		}
		void BMI088_SPI_OFF(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
		}
		void	BMI088_writeByte(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t subAddress, uint8_t data);
		void 	BMI088_readBytes(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t subAddress, uint8_t len, uint8_t* spi_rev_buf);
		void  selftext_error_reset(void);
        void DWT_Init(void);
        float DWT_Get_time(void);
};


#endif


/**************************************PWM_MCL***********************************************/
#ifdef __TIM_H__


class MCL_snail
{
	public:
		TIM_HandleTypeDef *		htim;
		TIM_HandleTypeDef *		htim_z;
		uint32_t 				Channel_z;
		TIM_HandleTypeDef *		htim_y;
		uint32_t 				Channel_y;
	
		void 					Init(void);
		void 					Init_XC_Calibration(uint8_t speed_z_max,uint8_t speed_y_max);
		void 					Init_Change_Steer(uint8_t dir,uint8_t speed_max);
		void 					stop(void);
		void 					run(uint8_t grade);
		HAL_StatusTypeDef 		shoot_state(void);
		void 					state_tick(TIM_HandleTypeDef *p);
		void					set_speed(uint8_t speed_z,uint8_t speed_y);
		
		MCL_snail(	TIM_HandleTypeDef *htim,
					TIM_HandleTypeDef *htim_z,uint32_t Channel_z,
					TIM_HandleTypeDef *htim_y,uint32_t Channel_y,
					uint8_t grade_1,uint8_t grade_2,uint8_t grade_3,
					uint8_t grade_1_error,uint8_t grade_2_error,uint8_t grade_3_error,
					const char *string_check
				 ):
					htim(htim),htim_z(htim_z),Channel_z(Channel_z),htim_y(htim_y),Channel_y(Channel_y),
					grade_1(grade_1),grade_2(grade_2),grade_3(grade_3),
					grade_1_error(grade_1_error),grade_2_error(grade_2_error),grade_3_error(grade_3_error),
					string_check(string_check){}
	
	private:
		uint8_t 		grade_1,grade_2,grade_3;
		uint8_t 		grade_1_error,grade_2_error,grade_3_error;
		const char 	*	string_check;
		uint8_t 		shoot_state_byte,run_stete;
		uint32_t 		time_20ms;
		uint8_t 		first_state;
};
#endif


/*************************************UD_check**********************************************/
typedef enum{
	UpDown_check_nothing,
	UpDown_check_falling,
	UpDown_check_rising
}UpDown_check_state;
class UpDown_check_class
{
	public:
		UpDown_check_class(bool initial_conditions):bit(initial_conditions){}
		UpDown_check_state updata(bool Condition)
		{
			if (((Condition) != 0) && ((bit & 1) == 0)) {
				bit |= 1;
				return UpDown_check_rising;
			}
			else if (!((Condition) != 0) && ((bit & 1) != 0)) {
				bit &= ~1;
				return UpDown_check_falling;
			}
			else return UpDown_check_nothing;
		}
	private:
		bool bit;
};		
		

/*************************************RGB**********************************************/
#ifdef __TIM_H__

#define PIXEL_NUM  5   										//灯珠数
#define NUM 			 (24*PIXEL_NUM + 300)    // Reset 280us（复位时间） / 1.25us = 224   NUM的值为单个灯珠的位宽（24）* 灯珠数量（PIXEL_NUM）+ 复位脉冲数    1/84M = 11.9ns
#define WS1  			 75    //重装值105
#define WS0  			 30

#define LED1       0
#define LED2       1
#define LED3       2
#define LED4       3
#define LED5       4

typedef struct {
	
uint8_t DC_flag;
uint8_t FeiPo_flag;	
uint8_t XTL_flag;
uint8_t JianSu_flag;	

}STATE_TYPEDEF;


class RGB_UI
{
		public:
			TIM_HandleTypeDef *		htim;
			uint32_t 				Channel;
		  STATE_TYPEDEF   state;
	  	void RGB_UI_Init(void);
			void WS_Load(void);
			void WS_WriteAll_RGB(uint8_t n_R, uint8_t n_G, uint8_t n_B);
			void WS_CloseAll(void);
			void WS281x_SetPixelRGB(uint16_t n ,uint8_t red, uint8_t green, uint8_t blue);

		RGB_UI(TIM_HandleTypeDef *htim,uint32_t Channel,const char *string_check_rgb_ui):
				htim(htim),Channel(Channel),string_check_rgb_ui(string_check_rgb_ui){}
		private:
			uint16_t send_Buf[NUM];
			const char*		string_check_rgb_ui;
			uint32_t WS281x_Color(uint8_t red, uint8_t green, uint8_t blue);
			void WS281x_SetPixelColor(uint16_t n, uint32_t GRBColor);
		
};
#endif

#endif
