#ifndef MY_MATH_H
#define MY_MATH_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stdint.h"
#include "math.h"
	 
	 
#define LIMIT(x,min,max) ((x)<(min)?(min):((x)>(max)?(max):(x)))

	 typedef struct 
{
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.005
    float R;//观测噪声协方差 初始化值为0.543
}KFP;//Kalman Filter parameter


typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
    float B;
    float Q;
    float R;
    float H;
	
}extKalman_t;

/***************/
/*数据结构*/

typedef struct
{
    uint16_t nowLength;
    uint16_t queueLength;
    float queueTotal;
    //长度
    float queue[100];
    //指针
    float aver_num;//平均值

    float Diff;//差分值

    uint8_t full_flag;
} QueueObj;


class Kf{

private:
	double x_last;
	double p_last;
public:
	double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,uint8_t kind);

};

typedef struct {
    QueueObj speed_queue;
    QueueObj accel_queue;
    QueueObj dis_queue;
	  float predict_angle;
	  float predict_angle_limt;
    float feedforwaurd_angle;
	  float speed_get;
	  float speed_get_last; 
	  float accel_get;
		float angle_now;
  	float angle_past;
	  float angle_get;
	  float fly_accel_get;
	  float distend_get;
	  float vy,vx,fly_t;
	  float dir_accel;
	  uint8_t eeror;
	  float YawTarget_now;
	  uint32_t 		rx_time_prev;		// 接收数据的前一时刻
    uint32_t 		rx_time_now;		// 接收数据的当前时刻
    uint16_t 		rx_time_fps;		// 帧率
} Vision_process_t;

extern Vision_process_t Vision_process;

extern extKalman_t kalman_accel,kalman_speedYaw,kalman_targetYaw,kalman_buffPitch;

	
void kalmanCreate(KFP *kfp,float T_Q,float T_R);
float kalmanFilter(KFP *kfp,float input);

void  Filter_IIRLPF(float in, float *out, float LpfAttFactor);	 
float Slope(float M ,float *queue ,uint16_t len);	 
void  KalmanCreate(extKalman_t *p,float T_Q,float T_R);
float KalmanFilter(extKalman_t *p,float dat);
float Get_Diff(uint8_t queue_len, QueueObj *Data,float add_data);
void  Vision_Normal(float angle);	 
float DeathZoom(float input, float center, float death);	 
float  Vision_Normal_NEW(float angle);	 
	 
void	MeanFilter_Init();
float MeanFilter(float num);	 
	 
typedef struct moving_Average_Filter
{
	float num[100];
	uint8_t lenth;
	uint8_t pot;//当前位置
	float total;
	float aver_num;
}moving_Average_Filter;	//最大设置MAF_MaxSize个
	 
	 #ifdef __cplusplus
}
#endif

#endif


