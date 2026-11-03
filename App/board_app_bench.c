#include "board_app.h"

#ifdef APP_BENCH

#include <stdio.h>

#include "bench_input.h"
#include "bsp_can.h"
#include "can.h"
#include "steering_wheel_app.h"
#include "usart.h"

#define APP_LED_MASTER_PIN GPIO_PIN_4
#define APP_LED_VESC_PIN   GPIO_PIN_5
#define APP_LED_ENABLE_PIN GPIO_PIN_6
#define APP_LED_GM_PIN     GPIO_PIN_7

SteeringWheelApp_t g_steering_wheel_app;
AppSnapshot_t g_app_snapshot;

static volatile uint32_t g_app_ms;
static volatile uint8_t g_uart_busy;
static char g_uart_buffer[384];
static uint32_t g_last_telemetry_ms;
static uint32_t g_last_led_ms;
static uint8_t g_was_active;

static void App_UpdateLeds(void)
{
    uint8_t heartbeat;
    uint8_t enabled;

    if (g_app_ms < APP_SELF_TEST_MS)
    {
        HAL_GPIO_WritePin(
            GPIOB,
            APP_LED_MASTER_PIN |
                APP_LED_VESC_PIN |
                APP_LED_ENABLE_PIN |
                APP_LED_GM_PIN,
            GPIO_PIN_RESET);
        return;
    }

    heartbeat = (((g_app_ms / 500U) % 2U) == 0U) ? 1U : 0U;
    enabled = g_app_snapshot.control_enabled;

    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_MASTER_PIN,
                      heartbeat ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_VESC_PIN,
                      g_app_snapshot.vesc_online ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_ENABLE_PIN,
                      enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_GM_PIN,
                      g_app_snapshot.gm6020_online ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void App_SendTelemetry(void)
{
    int length;

    if (g_uart_busy)
    {
        return;
    }

    length = snprintf(
        g_uart_buffer,
        sizeof(g_uart_buffer),
        "%u,%u,%u,%u,%u,%u,%u,"
        "%d,%d,%d,%d,%d,%d,%d,%d,"
        "%d,%d,%d,%d,%d,"
        "%u,%u,%u,%u,%u\r\n",
        (unsigned)g_app_ms,
        (unsigned)g_app_snapshot.control_enabled,
        (unsigned)g_app_snapshot.gm6020_online,
        (unsigned)g_app_snapshot.vesc_online,
        (unsigned)g_app_snapshot.need_fresh_control,
        (unsigned)g_app_snapshot.angle_reversed,
        (unsigned)g_bench_input.mode,
        (int)(g_bench_input.generated_angle_deg * 100.0f),
        (int)g_bench_input.generated_erpm,
        (int)(g_app_snapshot.mapped_target_deg * 100.0f),
        (int)(g_app_snapshot.final_target_angle_deg * 100.0f),
        (int)g_app_snapshot.final_target_erpm,
        (int)(g_steering_wheel_app.gm6020.angle_deg * 100.0f),
        (int)g_steering_wheel_app.gm6020.speed_rpm,
        (int)g_steering_wheel_app.gm6020.target_speed_rpm,
        (int)g_steering_wheel_app.gm6020.target_voltage,
        (int)g_steering_wheel_app.gm6020.current_raw,
        (int)g_steering_wheel_app.vesc.actual_erpm,
        (int)(g_steering_wheel_app.vesc.actual_current_a * 1000.0f),
        (unsigned)g_bench_input.enable,
        (unsigned)g_app_snapshot.applied_config_seq,
        (unsigned)g_app_snapshot.config_apply_status,
        (unsigned)g_app_snapshot.fault_flags,
        (int)(SetpointRamp_GetOutput(&g_steering_wheel_app.ramp) * 100.0f),
        (int)SetpointRamp_GetVelocity(&g_steering_wheel_app.ramp));

    if ((length > 0) &&
        ((uint32_t)length < sizeof(g_uart_buffer)) &&
        (HAL_UART_Transmit_IT(
             &huart1,
             (uint8_t *)g_uart_buffer,
             (uint16_t)length) == HAL_OK))
    {
        g_uart_busy = 1U;
    }
}

void BoardApp_Init(void)
{
    AppConfig_t defaults;

    g_app_ms = 0U;
    g_uart_busy = 0U;
    g_last_telemetry_ms = 0U;
    g_last_led_ms = 0U;
    g_was_active = 0U;

    AppConfig_LoadDefaults(&defaults);
    AppConfig_InitTuning(&defaults);
    BenchInput_Reset(&defaults);
    SteeringWheelApp_Init(
        &g_steering_wheel_app,
        &hcan1,
        &defaults,
        NULL,
        NULL);
    SteeringWheelApp_GetSnapshot(
        &g_steering_wheel_app,
        &g_app_snapshot);

    if (BSP_CAN_StartAcceptAll(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(USART1_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void BoardApp_OnCanRx(CAN_HandleTypeDef *hcan,
                      const CAN_RxHeaderTypeDef *header,
                      const uint8_t data[8])
{
    if (hcan == &hcan1)
    {
        SteeringWheelApp_OnCan1Rx(
            &g_steering_wheel_app,
            header,
            data,
            g_app_ms);
    }
}

void BoardApp_ControlStep(void)
{
    AppInputSnapshot_t input;
    const AppConfig_t *config;

    g_app_ms++;
    config = SteeringWheelApp_GetEffectiveConfig(
        &g_steering_wheel_app);
    BenchInput_Build(
        config,
        g_steering_wheel_app.gm6020.angle_deg,
        &input,
        g_app_ms);
    SteeringWheelApp_ControlStep(
        &g_steering_wheel_app,
        &input,
        g_app_ms);
    if (input.has_new_target == 0U)
    {
        BenchInput_TargetConsumed();
    }
    SteeringWheelApp_GetSnapshot(
        &g_steering_wheel_app,
        &g_app_snapshot);

    if (g_was_active && !g_app_snapshot.control_enabled)
    {
        BenchInput_ForceRearm();
    }
    g_was_active = g_app_snapshot.control_enabled;
}

void BoardApp_MainLoop(void)
{
    if ((uint32_t)(g_app_ms - g_last_led_ms) >= APP_LED_UPDATE_MS)
    {
        g_last_led_ms = g_app_ms;
        App_UpdateLeds();
    }

    if ((uint32_t)(g_app_ms - g_last_telemetry_ms) >=
        APP_TELEMETRY_MS)
    {
        g_last_telemetry_ms = g_app_ms;
        App_SendTelemetry();
    }
}

void BoardApp_OnUartTxComplete(void)
{
    g_uart_busy = 0U;
}

#endif /* APP_BENCH */
