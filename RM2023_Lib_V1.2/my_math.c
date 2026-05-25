#include "my_math.h"
#include "RM_Lib.h"
#include "communication.h"
#include "stm32f4xx_hal.h"

bool move_state=0;
uint8_t time=0;
void Filter_IIRLPF(float in, float *out, float LpfAttFactor)
{
	float res ;
	*out = *out + LpfAttFactor*(in - *out);	
}

extern Kf  kalman_speedYaw1,kalman_accel1,kalman_distend1;

//滑动滤波
float Slope(float M ,float *queue ,uint16_t len)
{
	float sum=0;
	float res=0;
  
		//队列已满，FIFO。
		for(uint16_t i=0;i<len-1;i++)
		{
			queue[i] = queue[i+1];
			//更新队列
		}
		queue[len-1] = M;
    
	//更新完队列
	for(uint16_t j=0;j<len;j++)
	{
		sum+=queue[j];
	}
	res = sum/(len);
	
	return res;
}

void kalmanCreate(KFP *kfp,float T_Q,float T_R)
{
	  kfp-> LastP=0;//上次估算协方差 初始化值为0.02
    kfp-> Now_P=0;//当前估算协方差 初始化值为0
    kfp-> out=0;//卡尔曼滤波器输出 初始化值为0
    kfp-> Kg=0;//卡尔曼增益 初始化值为0
    kfp-> Q=T_Q;//过程噪声协方差 初始化值为0.005
    kfp-> R=T_R;//观测噪声协方差 初始化值为0.543
}

float kalmanFilter(KFP *kfp,float input)
{
     //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
     kfp->Now_P = kfp->LastP + kfp->Q;
     //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
     //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
}

// 卡尔曼
 void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}
 float KalmanFilter(extKalman_t *p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)     状态方程
    p->P_mid = p->A*p->P_last+p->Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q            观测方程
		p->kg = p->P_mid/(p->P_mid+p->R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)   更新卡尔曼增益
		p->X_now = p->X_mid + p->kg*(dat-p->X_mid);   //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))  修正估计值
    p->P_now = (1-p->kg)*p->P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)           更新后验估计协方差
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
		
    return p->X_now;							  //输出预测结果x(k|k)
}





/*******************************************预测*******************************/
/**
  * @brief 卡尔曼滤波函数
  * @other 	Q：过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R：测量噪声，R增大，动态响应变慢，收敛稳定性变好
  */
double Kf::KalmanFilter(const double ResrcData,double ProcessNoise_Q,double MeasureNoise_R,uint8_t kind)
{
   double R = MeasureNoise_R;
   double Q = ProcessNoise_Q;
   double x_mid;
   double x_now;
   double p_mid ;
   double p_now;
   double kg;
	
	 if(Vision_process.eeror==1)
	 {
		 p_last = 0;
     x_last = 0;
	 }
	 
   x_mid=x_last;
   p_mid=p_last+Q;
   kg=p_mid/(p_mid+R);
   x_now=x_mid+kg*(ResrcData-x_mid);

   p_now=(1-kg)*p_mid;
   p_last = p_now;
   x_last = x_now;
   return x_now;
}

float Get_Diff(uint8_t queue_len, QueueObj *Data,float add_data)
{
    Data->queueTotal-=Data->queue[Data->nowLength];
    Data->queueTotal+=add_data;

		
    Data->queue[Data->nowLength]=add_data;
	
    Data->nowLength++;
	
    if(Data->full_flag==0)//初始队列未满
    {
        Data->aver_num=Data->queueTotal/Data->nowLength;
    }
		else if(Data->full_flag == 1)
	  {
	    Data->aver_num=(Data->queueTotal)/queue_len;			
	  }
    if(Data->nowLength>=queue_len)
    {
        Data->nowLength=0;
        Data->full_flag=1;
    }

    Data->Diff=add_data - Data->aver_num;			
    return Data->Diff;
}

/**
* @brief 清空队列
* @param void
* @return void
*	以队列的逻辑
*/
void Clear_Queue(QueueObj* queue)
{
    for(uint16_t i=0; i<60; i++)
    {
      queue->queue[i]=0;
    }
    queue->nowLength = 0;
    queue->queueTotal = 0;
    queue->aver_num=0;
    queue->Diff=0;
    queue->full_flag=0;
}

void  Vision_Normal(float angle)
{
	static float acc_use = 1.f;  
  static float predic_use =2.f;//2
  float dir_factor;
	float res;
	
	Vision_process.rx_time_now=HAL_GetTick();
	Vision_process.rx_time_fps=Vision_process.rx_time_now-Vision_process.rx_time_prev;
	Vision_process.rx_time_prev=Vision_process.rx_time_now;
	
	Vision_process.speed_get_last = Get_Diff(3,&Vision_process.speed_queue,angle);
	Vision_process.speed_get_last = 40.0*(Vision_process.speed_get_last/Vision_process.rx_time_fps); //每毫秒
	Vision_process.speed_get = kalman_speedYaw1.KalmanFilter(Vision_process.speed_get_last,0.004,11,0);
  Vision_process.speed_get = LIMIT(Vision_process.speed_get , -2.0 , 2.0);
		
	Vision_process.accel_get = Get_Diff(3,&Vision_process.accel_queue,Vision_process.speed_get);	 /*新版获取加速度10*/
  Vision_process.accel_get = 40.0*(Vision_process.accel_get/Vision_process.rx_time_fps);//每毫秒
  Vision_process.accel_get = kalman_accel1.KalmanFilter(Vision_process.accel_get,0.004,11,0);
	Vision_process.accel_get = LIMIT(Vision_process.accel_get , -1.0 , 1.0);

	if( (Vision_process.speed_get * Vision_process.accel_get)>=0 )
  {
    dir_factor= 1.5f;
  }
  else
  {
    dir_factor= 2.5f;
  }
	
	if(isnan(Vision_process.speed_get)||isnan(Vision_process.accel_get)) //出错
	{
		move_state=1;
		Clear_Queue(&Vision_process.speed_queue);
    Clear_Queue(&Vision_process.accel_queue);
		Vision_process.eeror=1;
		Vision_process.feedforwaurd_angle = 0;
    Vision_process.predict_angle = 0;//清0预测角
		Vision_process.speed_get_last=0;
		Vision_process.accel_get=0;
		Vision_process.speed_get=0;
//		response.distance=0;
	}	
	else if((abs(Vision_process.speed_get)==2.0)&&(abs(Vision_process.accel_get)==1.0)&&(abs(Vision_process.predict_angle)==Vision_process.predict_angle_limt))//超范围
	{
		Clear_Queue(&Vision_process.speed_queue);
    Clear_Queue(&Vision_process.accel_queue);
		Vision_process.eeror=1;
		Vision_process.feedforwaurd_angle = 0;
    Vision_process.predict_angle = 0;//清0预测角
		Vision_process.speed_get_last=0;
		Vision_process.accel_get=0;
		Vision_process.speed_get=0;
//		response.distance=0;
	}
	else if((Vision_process.speed_get * Vision_process.accel_get)<=0)//反向
	{
		Clear_Queue(&Vision_process.speed_queue);
    Clear_Queue(&Vision_process.accel_queue);
		Vision_process.eeror=1;
		Vision_process.feedforwaurd_angle = 0;
    Vision_process.predict_angle = 0;//清0预测角
		Vision_process.speed_get_last=0;
		Vision_process.accel_get=0;
		Vision_process.speed_get=0;
//		response.distance=0;
	}
	else
	{
		if(request.shooter_speed_limit==16)
		{
			Vision_process.predict_angle_limt=3.8;
		}
		else
		{
			Vision_process.predict_angle_limt=4.3;//5.0
		}		
   Vision_process.feedforwaurd_angle = acc_use * Vision_process.accel_get; 	/*计算前馈角*/
//	 if(move_state&&(Vision_process.speed_get * Vision_process.accel_get)>0.1)
//	 {
//		 if(++time>10)
//		 {
//			 move_state=0;
//			 time=0;
//		 }
//		 Vision_process.predict_angle = predic_use * (1.5f*Vision_process.speed_get*response.distance+1.5f*dir_factor*Vision_process.feedforwaurd_angle*response.distance) ;//速度1.1，加速度3
//	 }
//		else
		 Vision_process.predict_angle = predic_use * (1.2f*Vision_process.speed_get*response.distance+1.2f*dir_factor*Vision_process.feedforwaurd_angle*response.distance) ;//速度1.1，加速度3
	 Vision_process.predict_angle = LIMIT(Vision_process.predict_angle , -Vision_process.predict_angle_limt, Vision_process.predict_angle_limt);
	}
}

/*********************************************************************************************************************/


#define Length   10
float buffer_num[Length];
float now_num;
float sum; 						/*<! 宽度和数字和 */
int flag,where_num;

void	MeanFilter_Init()
{
		static_assert((Length>0)&&(Length<101),"MedianFilter Length [1,100]");
		for(int x = 0 ; x < Length; x++) buffer_num[x] = 0;
		flag = Length;
		where_num = 0;
		sum = 0;
} 	
	
float MeanFilter(float num)
{
   now_num = num;
   sum -= buffer_num[where_num];			  /*<! sum减去旧值 */
   sum += num;													/*<! sum加上新值 */
   buffer_num[where_num++] = num;
   flag > 0? flag-- : 0;								/*<!flag=Length然后递减保证宽度内都是有效波值 */
   where_num %= Length; 
    
   if(flag>0)
     return now_num;
   else
     return (sum/Length);
}
		
		
float DeathZoom(float input, float center, float death)
{
	if(abs(input - center) < death)
		return center;
	return input;
}

/**
  * @brief    average_add
  * @note    滑动平均滤波器进入队列，先进先出
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_add(moving_Average_Filter *Aver, float add_data)
{
	
	Aver->total -=  Aver->num[Aver->pot];
	Aver->total += add_data;
	
	Aver->num[Aver->pot] = add_data;
	
	Aver->aver_num = (Aver->total)/(Aver->lenth);
	Aver->pot++;
	
	if(Aver->pot == Aver->lenth)
	{
		Aver->pot = 0;
	}
	
}
