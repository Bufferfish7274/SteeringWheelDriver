// Host integration tests for both production and debug builds.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "AppConfig.hpp"
#include "WheelApp.hpp"
#include "WheelAppApi.h"

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

static void Check(bool condition, const char *message)
{
    if (!condition)
    {
        std::printf("FAIL: %s\n", message);
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
    if (mailbox != nullptr)
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
    std::memcpy(data, g_rx_data, 8U);
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

#if !WHEEL_BUILD_DEBUG
static void InjectMasterControl(int16_t erpm, uint16_t angle_code)
{
    const uint16_t erpm_raw = static_cast<uint16_t>(erpm);

    g_rx_header.StdId = 0x101U;
    g_rx_header.ExtId = 0U;
    g_rx_header.IDE = CAN_ID_STD;
    g_rx_header.RTR = CAN_RTR_DATA;
    g_rx_header.DLC = 8U;
    g_rx_data[0] = static_cast<uint8_t>(erpm_raw >> 8U);
    g_rx_data[1] = static_cast<uint8_t>(erpm_raw);
    g_rx_data[2] = static_cast<uint8_t>(angle_code >> 8U);
    g_rx_data[3] = static_cast<uint8_t>(angle_code);
    g_rx_data[4] = 0U;
    g_rx_data[5] = 0U;
    g_rx_data[6] = 0U;
    g_rx_data[7] = 0U;
    g_rx_pending = 1U;
}
#endif

static void AdvanceMs(steering::WheelApp &app, uint32_t count)
{
    uint32_t i;

    for (i = 0U; i < count; i++)
    {
        app.onTimerElapsed(app.htim6);
    }
}

#if WHEEL_BUILD_DEBUG
static int RunDebugTests()
{
    steering::WheelApp &app = g_app;
    CAN_HandleTypeDef can1 = {&APP_STUB_CAN1};
    TIM_HandleTypeDef tim6 = {&APP_STUB_TIM6};
    UART_HandleTypeDef uart1 = {&APP_STUB_UART1};

    app.init(&can1, nullptr, &tim6, &uart1);
    app.start();

    Check(std::fabs(app.gm6020.position_pid.kp -
                    steering::config::Gm6020PosKp) < 0.001f,
          "GM6020 position config initialized once from AppConfig");
    Check(std::fabs(app.gm6020.speed_pid.ki -
                    steering::config::Gm6020SpeedKi) < 0.001f,
          "GM6020 speed config initialized once from AppConfig");
    Check(std::fabs(app.gm6020.position_pid.dt -
                    steering::config::ControlDtS) < 0.000001f,
          "GM6020 uses the unified control period");
    Check(std::fabs(app.target_ramp.max_speed_deg_s -
                    steering::config::RampMaxSpeedDegS) < 0.001f,
          "target ramp uses AppConfig speed limit");
    Check(std::fabs(app.target_ramp.max_accel_deg_s2 -
                    steering::config::RampMaxAccelDegS2) < 0.001f,
          "target ramp uses AppConfig acceleration limit");

    Check(g_can_start_count == 1U,
          "debug build starts CAN1 only");
    Check(g_timer_start_count == 1U,
          "debug build starts the control timer");

    AdvanceMs(app, steering::config::SelfTestMs + 1U);

    g_wheel_debug.mode = steering::WheelDebugMode::GmPositionStep;
    app.onTimerElapsed(&tim6);
    Check(g_wheel_debug.enable == 0U,
          "mode change clears debug enable");

    InjectGmFeedback(steering::config::Gm6020ZeroRaw, 0);
    app.onCanRx(&can1);
    Check(app.gm6020.is_online == 1U,
          "GM6020 feedback reaches App");

    g_wheel_debug.target_angle_deg = 30.0f;
    g_wheel_debug.direction_erpm = 0;
    g_wheel_debug.use_ramp = 0U;
    g_wheel_debug.enable = 1U;
    app.onTimerElapsed(&tim6);

    Check(app.gm6020.is_enabled == 1U,
          "position debug mode enables GM6020");
    Check(std::fabs(app.gm6020.target_angle_deg - 30.0f) < 0.01f,
          "position target reaches GM6020 driver");
    Check(app.vesc.is_enabled == 0U,
          "position debug mode keeps VESC disabled");

    g_wheel_debug.mode = steering::WheelDebugMode::GmSpeed;
    app.onTimerElapsed(&tim6);
    Check(app.gm6020.is_enabled == 0U,
          "speed mode transition disables GM6020");

    InjectGmFeedback(steering::config::Gm6020ZeroRaw, 0);
    app.onCanRx(&can1);
    g_wheel_debug.target_speed_rpm = 50.0f;
    g_wheel_debug.enable = 1U;
    app.onTimerElapsed(&tim6);
    Check(app.gm6020.control_mode ==
              steering::Gm6020::ControlMode::Speed,
          "speed mode selects GM6020 speed control");
    Check(std::fabs(app.gm6020.target_speed_rpm - 50.0f) < 0.001f,
          "speed target reaches GM6020 driver");

    g_wheel_debug.mode = steering::WheelDebugMode::GmVoltage;
    app.onTimerElapsed(&tim6);
    InjectGmFeedback(steering::config::Gm6020ZeroRaw, 0);
    app.onCanRx(&can1);
    g_wheel_debug.target_voltage_raw = 1000;
    g_wheel_debug.enable = 1U;
    app.onTimerElapsed(&tim6);
    Check(app.gm6020.control_mode ==
              steering::Gm6020::ControlMode::Voltage,
          "voltage mode selects GM6020 voltage control");
    Check(app.gm6020.target_voltage == 1000,
          "direct voltage reaches GM6020 driver");

    g_wheel_debug.mode = steering::WheelDebugMode::VescSpeed;
    app.onTimerElapsed(&tim6);
    InjectVescStatus(0);
    app.onCanRx(&can1);
    g_wheel_debug.target_erpm = 5000;
    g_wheel_debug.erpm_limit = 1000;
    g_wheel_debug.enable = 1U;
    app.onTimerElapsed(&tim6);
    Check(app.vesc.is_enabled == 1U,
          "VESC debug mode enables VESC");
    Check(app.vesc.target_erpm == 1000,
          "VESC debug target is limited");
    Check(app.gm6020.is_enabled == 0U,
          "VESC debug mode keeps GM6020 disabled");

    AdvanceMs(app, 101U);
    Check(g_wheel_debug.enable == 0U,
          "VESC timeout clears debug enable");
    Check(app.vesc.is_enabled == 0U,
          "VESC timeout disables actuator");

    if (g_failures != 0U)
    {
        std::printf("App debug host tests: %u failure(s)\n",
                    static_cast<unsigned>(g_failures));
        return 1;
    }

    std::printf("App debug host tests: all passed\n");
    return 0;
}

#else
static int RunProductionTests()
{
    steering::WheelApp &app = g_app;
    CAN_HandleTypeDef can1 = {&APP_STUB_CAN1};
    CAN_HandleTypeDef can2 = {&APP_STUB_CAN2};
    TIM_HandleTypeDef tim6 = {&APP_STUB_TIM6};
    UART_HandleTypeDef uart1 = {&APP_STUB_UART1};

    app.init(&can1, &can2, &tim6, &uart1);
    app.start();

    Check(g_can_start_count == 2U,
          "production build starts CAN1 and CAN2");
    Check(g_timer_start_count == 1U,
          "production build starts the control timer");
    Check(app.master.control_id == 0x101U,
          "master control ID comes from config");
    Check(app.master.heartbeat_id == 0x111U,
          "master heartbeat ID comes from config");

    AdvanceMs(app, steering::config::SelfTestMs + 1U);
    InjectGmFeedback(steering::config::Gm6020ZeroRaw, 0);
    app.onCanRx(&can1);
    InjectVescStatus(0);
    app.onCanRx(&can1);
    InjectMasterControl(400, 0U);
    app.onCanRx(&can2);
    app.onTimerElapsed(&tim6);

    Check(app.wheel_active == 1U,
          "fresh master control activates the wheel");
    Check(app.gm6020.is_enabled == 1U,
          "production control enables GM6020");
    Check(app.vesc.is_enabled == 1U,
          "production control enables VESC");
    Check(app.vesc.target_erpm != 0,
          "production control reaches VESC");

    AdvanceMs(app, steering::config::VescOnlineTimeoutMs + 1U);
    Check(app.wheel_active == 0U,
          "actuator timeout disables the wheel");
    Check(app.need_fresh_control == 1U,
          "recovery requires a fresh master command");

    if (g_failures != 0U)
    {
        std::printf("App production host tests: %u failure(s)\n",
                    static_cast<unsigned>(g_failures));
        return 1;
    }
    std::printf("App production host tests: all passed\n");
    return 0;
}
#endif

int main()
{
#if WHEEL_BUILD_DEBUG
    return RunDebugTests();
#else
    return RunProductionTests();
#endif
}
