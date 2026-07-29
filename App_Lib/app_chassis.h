#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"
#include "tim.h"
#include "usart.h"

void App_Chassis_Init(void);
void App_Chassis_Loop(void);
void App_Chassis_CAN1_RxFifo0Callback(CAN_HandleTypeDef *hcan);
void App_Chassis_CAN2_RxFifo1Callback(CAN_HandleTypeDef *hcan);
void App_Chassis_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void App_Chassis_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
