#include "WheelAppApi.h"

#include "WheelApp.hpp"

extern "C" void WheelApp_Init(
    CAN_HandleTypeDef *hcan1,
    CAN_HandleTypeDef *hcan2,
    TIM_HandleTypeDef *htim6,
    UART_HandleTypeDef *huart1)
{
    g_app.init(hcan1, hcan2, htim6, huart1);
}

extern "C" void WheelApp_Start(void)
{
    g_app.start();
}

extern "C" void WheelApp_Loop(void)
{
    g_app.loop();
}

extern "C" void WheelApp_OnCanRx(CAN_HandleTypeDef *hcan)
{
    g_app.onCanRx(hcan);
}

extern "C" void WheelApp_OnTimerElapsed(TIM_HandleTypeDef *htim)
{
    g_app.onTimerElapsed(htim);
}

extern "C" void WheelApp_OnUartTxComplete(
    UART_HandleTypeDef *huart)
{
    g_app.onUartTxComplete(huart);
}
