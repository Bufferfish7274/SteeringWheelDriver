#include "board_app.h"

#ifndef APP_BENCH

#include <stdio.h>

#include "MasterCan.h"
#include "bsp_can.h"
#include "can.h"
#include "steering_wheel_app.h"
#include "usart.h"

#define APP_LED_MASTER_PIN GPIO_PIN_4
#define APP_LED_VESC_PIN   GPIO_PIN_5
#define APP_LED_ENABLE_PIN GPIO_PIN_6
#define APP_LED_GM_PIN     GPIO_PIN_7

SteeringWheelApp_t g_steering_wheel_app;
MasterCan_t g_master;
AppSnapshot_t g_app_snapshot;

static volatile uint32_t g_app_ms;
static volatile uint8_t g_control_pending;
static volatile uint8_t g_uart_busy;
static char g_uart_buffer[384];
static uint32_t g_last_telemetry_ms;
static uint32_t g_last_led_ms;

static uint32_t App_AgeMs(uint32_t now_ms, uint32_t timestamp_ms)
{
    if (timestamp_ms == 0U)
    {
        return 0xFFFFFFFFU;
    }
    return (uint32_t)(now_ms - timestamp_ms);
}

static uint32_t App_GetFaultFlags(void)
{
    uint32_t flags = g_app_snapshot.fault_flags;

    if (!g_master.link_valid)
    {
        flags |= APP_FAULT_MASTER_LINK;
    }
    if (!g_master.control_seen)
    {
        flags |= APP_FAULT_NO_CONTROL;
    }
    return flags;
}

static void App_UpdateLeds(void)
{
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

    enabled =
        (g_app_snapshot.control_enabled &&
         g_steering_wheel_app.gm6020.is_enabled &&
         g_steering_wheel_app.vesc.is_enabled) ? 1U : 0U;

    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_MASTER_PIN,
                      g_master.link_valid ? GPIO_PIN_RESET : GPIO_PIN_SET);
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
        "%u,%u,%u,%u,%u,%u,%u,%u,"
        "%d,%u,%d,%d,%d,%d,"
        "%d,%d,%d,%u,%d,"
        "%d,%d,%d,%d,%d,"
        "%u,%u,%u\r\n",
        (unsigned)g_app_ms,
        (unsigned)g_master.link_valid,
        (unsigned)g_master.control_seen,
        (unsigned)g_app_snapshot.gm6020_online,
        (unsigned)g_app_snapshot.vesc_online,
        (unsigned)g_app_snapshot.control_enabled,
        (unsigned)g_app_snapshot.angle_reversed,
        (unsigned)g_app_snapshot.need_fresh_control,
        (int)g_master.drive_erpm,
        (unsigned)g_master.angle_code,
        (int)(g_master.target_angle_deg * 100.0f),
        (int)(g_app_snapshot.mapped_target_deg * 100.0f),
        (int)(g_app_snapshot.final_target_angle_deg * 100.0f),
        (int)g_app_snapshot.final_target_erpm,
        (int)(g_steering_wheel_app.gm6020.angle_deg * 100.0f),
        (int)g_steering_wheel_app.gm6020.speed_rpm,
        (int)g_steering_wheel_app.gm6020.current_raw,
        (unsigned)g_steering_wheel_app.gm6020.temperature,
        (int)g_steering_wheel_app.gm6020.target_voltage,
        (int)g_steering_wheel_app.vesc.actual_erpm,
        (int)(g_steering_wheel_app.vesc.actual_current_a * 1000.0f),
        (int)(g_steering_wheel_app.vesc.duty * 1000.0f),
        (int)(SetpointRamp_GetOutput(&g_steering_wheel_app.ramp) * 100.0f),
        (int)SetpointRamp_GetVelocity(&g_steering_wheel_app.ramp),
        (unsigned)App_AgeMs(g_app_ms, g_master.last_control_ms),
        (unsigned)App_AgeMs(g_app_ms, g_master.last_heartbeat_ms),
        (unsigned)App_GetFaultFlags());

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

static HAL_StatusTypeDef App_ReconfigureMasterBus(
    void *context,
    const AppConfig_t *config)
{
    (void)context;

    MasterCan_Init(&g_master, config->master_wheel_id);
    g_master.link_timeout_ms = config->master_link_timeout_ms;
    (void)HAL_CAN_Stop(&hcan2);
    return BSP_CAN_StartMasterFilter(
        &hcan2,
        config->master_wheel_id);
}

void BoardApp_Init(void)
{
    AppConfig_t defaults;

    g_app_ms = 0U;
    g_control_pending = 0U;
    g_uart_busy = 0U;
    g_last_telemetry_ms = 0U;
    g_last_led_ms = 0U;

    AppConfig_LoadDefaults(&defaults);
    AppConfig_InitTuning(&defaults);
    SteeringWheelApp_Init(
        &g_steering_wheel_app,
        &hcan1,
        &defaults,
        App_ReconfigureMasterBus,
        NULL);
    MasterCan_Init(&g_master, defaults.master_wheel_id);
    SteeringWheelApp_GetSnapshot(
        &g_steering_wheel_app,
        &g_app_snapshot);

    if (BSP_CAN_StartAcceptAll(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (BSP_CAN_StartMasterFilter(
            &hcan2,
            defaults.master_wheel_id) != HAL_OK)
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
    else if ((hcan == &hcan2) &&
             (header != NULL) &&
             (header->IDE == CAN_ID_STD) &&
             (header->RTR == CAN_RTR_DATA) &&
             (header->DLC == 8U))
    {
        MasterCan_OnRx(&g_master, header, data, g_app_ms);
        if (header->StdId == g_master.control_id)
        {
            g_control_pending = 1U;
        }
    }
}

void BoardApp_ControlStep(void)
{
    AppInputSnapshot_t input;
    const AppConfig_t *config;

    g_app_ms++;
    config = SteeringWheelApp_GetEffectiveConfig(
        &g_steering_wheel_app);
    g_master.link_timeout_ms = config->master_link_timeout_ms;
    MasterCan_Update(&g_master, g_app_ms);

    input.kind = APP_COMMAND_MOTION;
    input.wheel_target_angle_deg =
        SteeringWheelApp_MapMasterAngle(
            config,
            g_master.target_angle_deg);
    input.drive_erpm = g_master.drive_erpm;
    input.gm6020_target_rpm = 0.0f;
    input.gm6020_target_voltage = 0;
    input.vesc_target_erpm = 0;
    input.source_ready =
        (g_master.link_valid && g_master.control_seen) ? 1U : 0U;
    input.has_new_target = g_control_pending;
    input.required_actuators = APP_ACTUATOR_ALL;

    SteeringWheelApp_ControlStep(
        &g_steering_wheel_app,
        &input,
        g_app_ms);
    g_control_pending = input.has_new_target;
    SteeringWheelApp_GetSnapshot(
        &g_steering_wheel_app,
        &g_app_snapshot);
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

#endif /* !APP_BENCH */
