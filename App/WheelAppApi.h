#ifndef WHEEL_APP_API_H
#define WHEEL_APP_API_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void WheelApp_Init(CAN_HandleTypeDef *hcan1,
                   CAN_HandleTypeDef *hcan2,
                   TIM_HandleTypeDef *htim6,
                   UART_HandleTypeDef *huart1);
void WheelApp_Start(void);
void WheelApp_Loop(void);
void WheelApp_OnCanRx(CAN_HandleTypeDef *hcan);
void WheelApp_OnTimerElapsed(TIM_HandleTypeDef *htim);
void WheelApp_OnUartTxComplete(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
