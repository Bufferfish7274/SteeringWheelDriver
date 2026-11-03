#ifndef BOARD_APP_H
#define BOARD_APP_H

#include <stdint.h>

#include "main.h"

/** @brief 初始化当前编译变体并启动对应的 CAN 总线。 */
void BoardApp_Init(void);

/** @brief 处理一条已经读取的 CAN 报文。 */
void BoardApp_OnCanRx(CAN_HandleTypeDef *hcan,
                      const CAN_RxHeaderTypeDef *header,
                      const uint8_t data[8]);

/** @brief 1 kHz 控制入口。 */
void BoardApp_ControlStep(void);

/** @brief 主循环低频调度入口。 */
void BoardApp_MainLoop(void);

/** @brief UART 非阻塞发送完成通知。 */
void BoardApp_OnUartTxComplete(void);

#endif /* BOARD_APP_H */
