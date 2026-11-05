#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "App.h"
#include "AppConfig.h"

const int APP_STUB_CAN1 = 1;
const int APP_STUB_CAN2 = 2;
const int APP_STUB_TIM6 = 3;
const int APP_STUB_UART1 = 4;
int APP_STUB_GPIOB = 5;

static uint32_t g_failures;
static uint32_t g_can_start_count;
static uint32_t g_timer_start_count;
static uint8_t g_rx_pending;
static CAN_RxHeaderTypeDef g_rx_header;
static uint8_t g_rx_data[8];

static void Check(int condition, const char *message)
{
    if (!condition)
    {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

void Error_Handler(void)
{
    g_failures++;
}

HAL_StatusTypeDef HAL_CAN_ConfigFilter(
    CAN_HandleTypeDef *hcan,
    CAN_FilterTypeDef *filter)
{
    (void)hcan;
    (void)filter;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    g_can_start_count++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_ActivateNotification(
    CAN_HandleTypeDef *hcan,
    uint32_t notification)
{
    (void)hcan;
    (void)notification;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    CAN_HandleTypeDef *hcan,
    CAN_TxHeaderTypeDef *header,
    uint8_t data[8],
    uint32_t *mailbox)
{
    (void)hcan;
    (void)header;
    (void)data;
    if (mailbox != NULL)
    {
        *mailbox = 0U;
    }
    return HAL_OK;
}

uint32_t HAL_CAN_GetRxFifoFillLevel(
    CAN_HandleTypeDef *hcan,
    uint32_t fifo)
{
    (void)hcan;
    (void)fifo;
    return g_rx_pending ? 1U : 0U;
}

HAL_StatusTypeDef HAL_CAN_GetRxMessage(
    CAN_HandleTypeDef *hcan,
    uint32_t fifo,
    CAN_RxHeaderTypeDef *header,
    uint8_t data[8])
{
    (void)hcan;
    (void)fifo;
    if (!g_rx_pending)
    {
        return HAL_ERROR;
    }
    *header = g_rx_header;
    memcpy(data, g_rx_data, 8U);
    g_rx_pending = 0U;
    return HAL_OK;
}

uint32_t HAL_CAN_GetTxMailboxesFreeLevel(
    CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    return 3U;
}

uint32_t HAL_CAN_GetState(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    return 0U;
}

uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    return 0U;
}

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(
    TIM_HandleTypeDef *htim)
{
    (void)htim;
    g_timer_start_count++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size)
{
    (void)huart;
    (void)data;
    (void)size;
    return HAL_OK;
}

void HAL_GPIO_WritePin(
    GPIO_TypeDef *port,
    uint16_t pin,
    GPIO_PinState state)
{
    (void)port;
    (void)pin;
    (void)state;
}

void HAL_NVIC_SetPriority(
    int irq,
    uint32_t priority,
    uint32_t subpriority)
{
    (void)irq;
    (void)priority;
    (void)subpriority;
}

void HAL_NVIC_EnableIRQ(int irq)
{
    (void)irq;
}

static void InjectGmFeedback(uint32_t raw_angle, int16_t speed_rpm)
{
    g_rx_header.StdId = 0x205U;
    g_rx_header.ExtId = 0U;
    g_rx_header.IDE = CAN_ID_STD;
    g_rx_header.RTR = CAN_RTR_DATA;
    g_rx_header.DLC = 8U;
    g_rx_data[0] = (uint8_t)(raw_angle >> 8U);
    g_rx_data[1] = (uint8_t)raw_angle;
    g_rx_data[2] = (uint8_t)((uint16_t)speed_rpm >> 8U);
    g_rx_data[3] = (uint8_t)speed_rpm;
    g_rx_data[4] = 0U;
    g_rx_data[5] = 0U;
    g_rx_data[6] = 25U;
    g_rx_data[7] = 0U;
    g_rx_pending = 1U;
}

static void InjectVescStatus(int32_t erpm)
{
    uint32_t raw_erpm = (uint32_t)erpm;

    g_rx_header.StdId = 0U;
    g_rx_header.ExtId = 0x943U;
    g_rx_header.IDE = CAN_ID_EXT;
    g_rx_header.RTR = CAN_RTR_DATA;
    g_rx_header.DLC = 8U;
    g_rx_data[0] = (uint8_t)(raw_erpm >> 24U);
    g_rx_data[1] = (uint8_t)(raw_erpm >> 16U);
    g_rx_data[2] = (uint8_t)(raw_erpm >> 8U);
    g_rx_data[3] = (uint8_t)raw_erpm;
    g_rx_data[4] = 0U;
    g_rx_data[5] = 0U;
    g_rx_data[6] = 0U;
    g_rx_data[7] = 0U;
    g_rx_pending = 1U;
}

static void AdvanceMs(App_t *app, uint32_t count)
{
    uint32_t i;

    for (i = 0U; i < count; i++)
    {
        App_OnTimerElapsed(app, app->htim6);
    }
}

int main(void)
{
    App_t app;
    CAN_HandleTypeDef can1 = {&APP_STUB_CAN1};
    TIM_HandleTypeDef tim6 = {&APP_STUB_TIM6};
    UART_HandleTypeDef uart1 = {&APP_STUB_UART1};

    App_Init(&app, &can1, NULL, &tim6, &uart1);
    App_Start(&app);

    Check(g_can_start_count == 1U,
          "debug build starts CAN1 only");
    Check(g_timer_start_count == 1U,
          "debug build starts the control timer");

    AdvanceMs(&app, APP_SELF_TEST_MS + 1U);

    g_wheel_debug.mode = WHEEL_DEBUG_GM_POSITION_STEP;
    App_OnTimerElapsed(&app, &tim6);
    Check(g_wheel_debug.enable == 0U,
          "mode change clears debug enable");

    InjectGmFeedback(APP_GM6020_ZERO_RAW, 0);
    App_OnCanRx(&app, &can1);
    Check(app.gm6020.is_online == 1U,
          "GM6020 feedback reaches App");

    g_wheel_debug.target_angle_deg = 30.0f;
    g_wheel_debug.direction_erpm = 0;
    g_wheel_debug.use_ramp = 0U;
    g_wheel_debug.enable = 1U;
    App_OnTimerElapsed(&app, &tim6);

    Check(app.gm6020.is_enabled == 1U,
          "position debug mode enables GM6020");
    Check(fabsf(app.gm6020.target_angle_deg - 30.0f) < 0.01f,
          "position target reaches GM6020 driver");
    Check(app.vesc.is_enabled == 0U,
          "position debug mode keeps VESC disabled");

    g_wheel_debug.mode = WHEEL_DEBUG_GM_SPEED;
    App_OnTimerElapsed(&app, &tim6);
    Check(app.gm6020.is_enabled == 0U,
          "speed mode transition disables GM6020");

    InjectGmFeedback(APP_GM6020_ZERO_RAW, 0);
    App_OnCanRx(&app, &can1);
    g_wheel_debug.target_speed_rpm = 50.0f;
    g_wheel_debug.enable = 1U;
    App_OnTimerElapsed(&app, &tim6);
    Check(app.gm6020.control_mode == GM6020_CONTROL_SPEED,
          "speed mode selects GM6020 speed control");
    Check(fabsf(app.gm6020.target_speed_rpm - 50.0f) < 0.001f,
          "speed target reaches GM6020 driver");

    g_wheel_debug.mode = WHEEL_DEBUG_GM_VOLTAGE;
    App_OnTimerElapsed(&app, &tim6);
    InjectGmFeedback(APP_GM6020_ZERO_RAW, 0);
    App_OnCanRx(&app, &can1);
    g_wheel_debug.target_voltage_raw = 1000;
    g_wheel_debug.enable = 1U;
    App_OnTimerElapsed(&app, &tim6);
    Check(app.gm6020.control_mode == GM6020_CONTROL_VOLTAGE,
          "voltage mode selects GM6020 voltage control");
    Check(app.gm6020.target_voltage == 1000,
          "direct voltage reaches GM6020 driver");

    g_wheel_debug.mode = WHEEL_DEBUG_VESC_SPEED;
    App_OnTimerElapsed(&app, &tim6);
    InjectVescStatus(0);
    App_OnCanRx(&app, &can1);
    g_wheel_debug.target_erpm = 5000;
    g_wheel_debug.erpm_limit = 1000;
    g_wheel_debug.enable = 1U;
    App_OnTimerElapsed(&app, &tim6);
    Check(app.vesc.is_enabled == 1U,
          "VESC debug mode enables VESC");
    Check(app.vesc.target_erpm == 1000,
          "VESC debug target is limited");
    Check(app.gm6020.is_enabled == 0U,
          "VESC debug mode keeps GM6020 disabled");

    AdvanceMs(&app, 101U);
    Check(g_wheel_debug.enable == 0U,
          "VESC timeout clears debug enable");
    Check(app.vesc.is_enabled == 0U,
          "VESC timeout disables actuator");

    if (g_failures != 0U)
    {
        printf("App debug host tests: %u failure(s)\n",
               (unsigned)g_failures);
        return 1;
    }

    printf("App debug host tests: all passed\n");
    return 0;
}
