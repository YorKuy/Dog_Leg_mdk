#include "communication.h"

uc Mini_PC_info_ubuf[MINI_PC_BUF_SIZE];
uc Mini_PC_tx_buf[128];
uc Mini_PC_rx_buf[128];

//数据收发单体
volatile _request request;
volatile _response  response;

uc Mini_PC_Step = 0;

uc cal_crc_table(uc *ptr, uc len) 
{
    uc crc = 0x00;
 	
    while (len--)
    {
        crc = crc_table[crc ^ *ptr++];
    }
    return (crc);
}

#if Communication_Mode == Communication_huart

void Mini_PC_Init(void)
{
	__HAL_UART_ENABLE_IT(&MINI_PC_USART_HANDLE, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&MINI_PC_USART_HANDLE,Mini_PC_rx_buf,128);
}

void Mini_PC_SendData()
{
	Mini_PC_tx_buf[0] = 0x38;	// SOF
	Mini_PC_tx_buf[1] = 0x00;	// 清零

	Mini_PC_tx_buf[1] |= (request.close_PC_status << 3) & 0x08;		//1   关机
	Mini_PC_tx_buf[1] |= (request.buff_status << 2) & 0x04;			  //1   符
	Mini_PC_tx_buf[1] |= (request.adjust_camera << 1) & 0x02;			//1    标定相机
	Mini_PC_tx_buf[1] |= (request.mine << 0) & 0x01;						  //    红蓝方
	
	Mini_PC_tx_buf[2] =  request.shooter_speed_limit;  

	Mini_PC_tx_buf[3]=	request.Yaw_sensor_data_mang.c[0];
	Mini_PC_tx_buf[4]=	request.Yaw_sensor_data_mang.c[1];	
	Mini_PC_tx_buf[5]=	request.Yaw_sensor_data_mang.c[2];
	Mini_PC_tx_buf[6]=	request.Yaw_sensor_data_mang.c[3];	
	
	Mini_PC_tx_buf[7]=	request.pitch_mang.c[0];
	Mini_PC_tx_buf[8]=	request.pitch_mang.c[1];	
	Mini_PC_tx_buf[9]=	request.pitch_mang.c[2];
	Mini_PC_tx_buf[10]=	request.pitch_mang.c[3];
	
	Mini_PC_tx_buf[11] = cal_crc_table(Mini_PC_tx_buf,11);
	HAL_UART_Transmit_DMA(&MINI_PC_USART_HANDLE,Mini_PC_tx_buf,12);
}

#elif Communication_Mode == Communication_USB_VCP

void Mini_PC_SendData()
{
	Mini_PC_tx_buf[0] = 0x38;	// SOF
	Mini_PC_tx_buf[1] = 0x00;	// 清零

	Mini_PC_tx_buf[1] |= (request.close_PC_status << 3) & 0x08;		//1   关机
	Mini_PC_tx_buf[1] |= (request.buff_status << 2) & 0x04;			  //1   符
	Mini_PC_tx_buf[1] |= (request.adjust_camera << 1) & 0x02;			//1    标定相机
	Mini_PC_tx_buf[1] |= (request.mine << 0) & 0x01;						  //    红蓝方
	
	Mini_PC_tx_buf[2] =  request.shooter_speed_limit;  

	Mini_PC_tx_buf[3]=	request.Yaw_sensor_data_mang.c[0];
	Mini_PC_tx_buf[4]=	request.Yaw_sensor_data_mang.c[1];	
	Mini_PC_tx_buf[5]=	request.Yaw_sensor_data_mang.c[2];
	Mini_PC_tx_buf[6]=	request.Yaw_sensor_data_mang.c[3];	
	
	Mini_PC_tx_buf[7]=	request.pitch_mang.c[0];
	Mini_PC_tx_buf[8]=	request.pitch_mang.c[1];	
	Mini_PC_tx_buf[9]=	request.pitch_mang.c[2];
	Mini_PC_tx_buf[10]=	request.pitch_mang.c[3];
	
	Mini_PC_tx_buf[11] = cal_crc_table(Mini_PC_tx_buf,11);
	CDC_Transmit_FS(Mini_PC_tx_buf,12);
}

#elif Communication_Mode == Communication_USB_HID

void Mini_PC_SendData()
{

	Mini_PC_tx_buf[0] = 0x38;	// SOF
	Mini_PC_tx_buf[1] = 0x00;	// 清零

	Mini_PC_tx_buf[1] |= (request.close_PC_status << 3) & 0x08;		//1   关机
	Mini_PC_tx_buf[1] |= (request.buff_status << 2) & 0x04;			  //1   符
	Mini_PC_tx_buf[1] |= (request.adjust_camera << 1) & 0x02;			//1    标定相机
	Mini_PC_tx_buf[1] |= (request.mine << 0) & 0x01;						  //    红蓝方
	
	Mini_PC_tx_buf[2] =  request.shooter_speed_limit;  

	Mini_PC_tx_buf[3]=	request.Yaw_sensor_data_mang.c[0];
	Mini_PC_tx_buf[4]=	request.Yaw_sensor_data_mang.c[1];	
	Mini_PC_tx_buf[5]=	request.Yaw_sensor_data_mang.c[2];
	Mini_PC_tx_buf[6]=	request.Yaw_sensor_data_mang.c[3];	
	
	Mini_PC_tx_buf[7]=	request.pitch_mang.c[0];
	Mini_PC_tx_buf[8]=	request.pitch_mang.c[1];	
	Mini_PC_tx_buf[9]=	request.pitch_mang.c[2];
	Mini_PC_tx_buf[10]=	request.pitch_mang.c[3];
	
	Mini_PC_tx_buf[11] = cal_crc_table(Mini_PC_tx_buf,11);
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,Mini_PC_tx_buf,12);
}

#endif

void getReceiveData(uc (*buf))
{
//	memcpy(&Mini_PC_rx_buf[11],buf,10);
	if(buf[0]==0x66 && buf[10]==cal_crc_table(buf,10))
	{
		response.pitch.c[0] = buf[1];
		response.pitch.c[1] = buf[2];
		response.pitch.c[2] = buf[3];
		response.pitch.c[3] = buf[4];
		response.yaw.c[0] = buf[5];
		response.yaw.c[1] = buf[6];
		response.yaw.c[2] = buf[7];
		response.yaw.c[3] = buf[8];
	 
		response.identify_target = buf[9];	//m
//		memcpy(&Mini_PC_rx_buf[0],buf,10);
//		response.identify_target = (buf[10]&0x01)>>0;
//		response.identify_buff = (buf[10]&0x02)>>1;
	}
	else return;
}
	
/************************************************Communication_KalmanFilter*****************************************************/
/**
	*	@brief Put the following code in funtion(USARTx_IRQHandler || CDC_Receive_FS || CUSTOM_HID_OutEvent_FS) of file(stm32fxxx_it.c || usbd_cdc_if.c || usbd_custom_hid_if.c):
**/
/*
		#include "my_math.h"   // USER CODE BEGIN INCLUDE
		

		extern Vision_process_t Vision_process;  
		extern Kf  kalman_speedYaw1,kalman_accel1,kalman_distend1;
		extern float lastupdate_cloud_yaw,update_cloud_yaw;	//记录视觉更新数据时的云台数据，给下次接收用


		extern float Pitch_goal,Yaw_goal;
		extern BMI088 BMI088_Yaw,BMI088_Pitch;
		extern uint16_t active_cnt, lost;
	
		getReceiveData(Buf);//or Mini_PC_rx_buf
	
		if(isnan(response.yaw.f)) response.yaw.f=0.0;
		if(isnan(response.pitch.f)) response.pitch.f=0.0;
			
		lastupdate_cloud_yaw=update_cloud_yaw;
		update_cloud_yaw =BMI088_Yaw.sensor_data.mang.z-response.yaw.f;

		active_cnt++;
		if(Vision_process.eeror==1)
		{
			lost++;
			active_cnt=0;
			Vision_process.feedforwaurd_angle = 0;
			Vision_process.predict_angle = 0;//清0预测角
			response.distance=0;
			Vision_process.accel_get=0;
			Vision_process.speed_get_last=0;
			Vision_process.speed_get=0;
			Vision_process.distend_get =0;
			Vision_process.speed_get = kalman_speedYaw1.KalmanFilter(Vision_process.speed_get_last,0,0,0);
			Vision_process.accel_get = kalman_accel1.KalmanFilter(Vision_process.accel_get,0,0,0);
			Vision_Normal(lastupdate_cloud_yaw);
		}			
		if(Buf[0]==0x66)//or Mini_PC_rx_buf[0]==0x66
		{	
			if(request.zimiao_status)
			{			
				if(YK.yaogan.s2==YK_SW_MID||YK.yaogan.s2==YK_SW_DOWN)
				{					
					if(request.buff_status)  //符
					{					
						Yaw_goal = BMI088_Yaw.sensor_data.mang.z-response.yaw.f;	//BMI088或者ADXRS453陀螺仪量与PID闭环的当前量一致，±号看实际来调，以下都是！！！！！
						Pitch_goal = BMI088_Pitch.sensor_data.mang.y-response.pitch.f;
					}					
					else //自瞄
					{
						if(active_cnt>150)
						{
							Vision_Normal(update_cloud_yaw);
						}
						else if(lost>100)
						{
							active_cnt=0;
							lost=0;
							Vision_process.eeror=0;
						}
						
						Yaw_goal = BMI088_Yaw.sensor_data.mang.z-response.yaw.f + Vision_process.predict_angle;//Vision_process.predict_angle是预测角
						Pitch_goal = BMI088_Pitch.sensor_data.mang.y-response.pitch.f;
					}
				}
			}
		}

// ********************************

//				Your code
	
// ********************************
	}

*/
/**
  *
**/

/***************************************************Communication_huart*********************************************************/
/**
	*	@brief Put the following code in funtion(USARTx_IRQHandler) of file(stm32fxxx_it.c):
**/
/*
	uint32_t tmp_flag = 0;
	uint32_t temp;
	tmp_flag =__HAL_UART_GET_FLAG(&MINI_PC_USART_HANDLE,UART_FLAG_IDLE); 
	if((tmp_flag != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&MINI_PC_USART_HANDLE);
		temp = MINI_PC_USART_HANDLE.Instance->SR;  
		temp = MINI_PC_USART_HANDLE.Instance->DR; 
		HAL_UART_DMAStop(&MINI_PC_USART_HANDLE); 
		getReceiveData(Mini_PC_rx_buf);
		HAL_UART_Receive_DMA(&MINI_PC_USART_HANDLE,Mini_PC_rx_buf,128);
		
// ********************************

//				Your code
	
// ********************************
	}

*/
/**
  *
**/


/*************************************************Communication_USB_HID********************************************************/
/**
	*	@brief Put the following code in funtion(CUSTOM_HID_ReportDesc_FS) of file(usbd_custom_hid_if.c):
**/
/*

	0x05,0x8c, // USAGE_PAGE (ST Page)
	0x09,0x01, // USAGE (Demo Kit)
	0xa1,0x01, // COLLECTION (Application

	// The Input report
	0x09,0x03, // USAGE ID - Vendor defined
	0x15,0x00, // LOGICAL_MINIMUM (0)
	0x26,0x00, 0xFF, // LOGICAL_MAXIMUM (255)
	0x75,0x08, // REPORT_SIZE (8bit)
	0x95,0x40, // REPORT_COUNT (64Byte)
	0x81,0x02, // INPUT (Data,Var,Abs)

	// The Output report
	0x09,0x04, // USAGE ID - Vendor defined
	0x15,0x00, // LOGICAL_MINIMUM (0)
	0x26,0x00,0xFF, // LOGICAL_MAXIMUM (255)
	0x75,0x08, // REPORT_SIZE (8bit)
	0x95,0x40, // REPORT_COUNT (64Byte)
	0x91,0x02, // OUTPUT (Data,Var,Abs)

// ********************************

//				Your code
	
// ********************************

*/
/**
  *
**/


/**
	*	@brief Put the following code in funtion(CUSTOM_HID_OutEvent_FS) of file(usbd_custom_hid_if.c):
**/
/*
	unsigned char USB_Received_Count;
	uint8_t i; //查看接收数据长
	USB_Received_Count = USBD_GetRxCount( &hUsbDeviceFS,CUSTOM_HID_EPOUT_ADDR );  //第一参数是USB句柄，第二个参数的是接收的末端地址；要获取发送的数据长度的话就把第二个参数改为发送末端地址即可
	USBD_CUSTOM_HID_HandleTypeDef   *hhid; //定义一个指向USBD_CUSTOM_HID_HandleTypeDef结构体的指针
	hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;//得到USB接收数据的储存地址
	
	for(i=0;i<USB_Received_Count;i++) 
	{
			Mini_PC_rx_buf[i]=hhid->Report_buf[i];  //把接收到的数据送到自定义的缓存区保存（Report_buf[i]为USB的接收缓存区）
	}
// ********************************

//				Your code
	
// ********************************

*/
/**
  *
**/







