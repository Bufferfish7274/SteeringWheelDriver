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
    const void *Instance;
} TIM_HandleTypeDef;

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

typedef struct
{
    uint32_t FilterBank;
    uint32_t FilterMode;
    uint32_t FilterScale;
    uint32_t FilterIdHigh;
    uint32_t FilterIdLow;
    uint32_t FilterMaskIdHigh;
    uint32_t FilterMaskIdLow;
    uint32_t FilterFIFOAssignment;
    uint32_t FilterActivation;
    uint32_t SlaveStartFilterBank;
} CAN_FilterTypeDef;

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET
} GPIO_PinState;

typedef int GPIO_TypeDef;

extern const int APP_STUB_CAN1;
extern const int APP_STUB_CAN2;
extern const int APP_STUB_TIM6;
extern const int APP_STUB_UART1;
extern int APP_STUB_GPIOB;

#define CAN1 (&APP_STUB_CAN1)
#define CAN2 (&APP_STUB_CAN2)
#define GPIOB (&APP_STUB_GPIOB)

#define CAN_ID_STD                    0U
#define CAN_ID_EXT                    1U
#define CAN_RTR_DATA                  0U
#define DISABLE                       0U
#define ENABLE                        1U
#define CAN_FILTERMODE_IDMASK         0U
#define CAN_FILTERSCALE_32BIT         0U
#define CAN_RX_FIFO0                  0U
#define CAN_IT_RX_FIFO0_MSG_PENDING   1U

#define GPIO_PIN_4                    0x0010U
#define GPIO_PIN_5                    0x0020U
#define GPIO_PIN_6                    0x0040U
#define GPIO_PIN_7                    0x0080U

#define USART1_IRQn                   37

HAL_StatusTypeDef HAL_CAN_ConfigFilter(
    CAN_HandleTypeDef *hcan,
    CAN_FilterTypeDef *filter);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(
    CAN_HandleTypeDef *hcan,
    uint32_t notification);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    CAN_HandleTypeDef *hcan,
    CAN_TxHeaderTypeDef *header,
    uint8_t data[8],
    uint32_t *mailbox);
uint32_t HAL_CAN_GetRxFifoFillLevel(
    CAN_HandleTypeDef *hcan,
    uint32_t fifo);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(
    CAN_HandleTypeDef *hcan,
    uint32_t fifo,
    CAN_RxHeaderTypeDef *header,
    uint8_t data[8]);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(
    CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_GetState(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *hcan);

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(
    TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_UART_Transmit_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size);
void HAL_GPIO_WritePin(
    GPIO_TypeDef *port,
    uint16_t pin,
    GPIO_PinState state);
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_EnableIRQ(int irq);

void Error_Handler(void);

#endif /* MAIN_H */
