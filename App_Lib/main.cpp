#include "main.h"
#include "app_chassis.h"
#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

extern "C" {
void SystemClock_Config(void);
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

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

  App_Chassis_Init();

  while (1)
  {
    App_Chassis_Loop();
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  App_Chassis_CAN1_RxFifo0Callback(hcan);
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  App_Chassis_CAN2_RxFifo1Callback(hcan);
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  App_Chassis_TIM_PeriodElapsedCallback(htim);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  App_Chassis_UART_RxCpltCallback(huart);
}
