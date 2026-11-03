#ifndef MAIN_H
#define MAIN_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    const void *Instance;
} CAN_HandleTypeDef;

typedef struct
{
    uint32_t reserved;
} GPIO_TypeDef;

typedef struct
{
    const void *Instance;
} UART_HandleTypeDef;

typedef struct
{
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
} CAN_RxHeaderTypeDef;

typedef struct
{
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
    uint32_t TransmitGlobalTime;
} CAN_TxHeaderTypeDef;

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

extern const int APP_STUB_CAN1;

#define CAN1 (&APP_STUB_CAN1)
#define CAN_ID_STD          0U
#define CAN_ID_EXT          1U
#define CAN_RTR_DATA        0U
#define CAN_RTR_REMOTE      1U
#define DISABLE             0U
#define GPIO_PIN_4          0x0010U
#define GPIO_PIN_5          0x0020U
#define GPIO_PIN_6          0x0040U
#define GPIO_PIN_7          0x0080U
#define GPIO_PIN_RESET      0U
#define GPIO_PIN_SET        1U
#define GPIOB               ((GPIO_TypeDef *)0x40020400U)
#define USART1_IRQn         37

HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    CAN_HandleTypeDef *hcan,
    CAN_TxHeaderTypeDef *header,
    uint8_t data[8],
    uint32_t *mailbox);

HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan);

HAL_StatusTypeDef HAL_UART_Transmit_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size);

void HAL_GPIO_WritePin(
    GPIO_TypeDef *port,
    uint16_t pin,
    uint32_t state);

void HAL_NVIC_SetPriority(uint32_t irq, uint32_t priority, uint32_t subpriority);

void HAL_NVIC_EnableIRQ(uint32_t irq);

void Error_Handler(void);

#endif /* MAIN_H */
