#include "WheelApp.hpp"

#include <cmath>

#include "BspCan.hpp"

namespace steering
{

namespace
{

constexpr SetpointRampConfig makeRampConfig()
{
    SetpointRampConfig config = {};
    config.max_speed_deg_s = config::kRampMaxSpeedDegS;
    config.max_accel_deg_s2 = config::kRampMaxAccelDegS2;
    return config;
}

constexpr Vesc::Config makeVescConfig()
{
    Vesc::Config config = {};
    config.controller_id = config::kVescControllerId;
    config.online_timeout_ms = config::kVescOnlineTimeoutMs;
    return config;
}

constexpr Gm6020::Config makeGm6020Config()
{
    Gm6020::Config config = {};
    config.id = config::kGm6020Id;
    config.zero_raw = config::kGm6020ZeroRaw;
    config.angle_sign = config::kGm6020AngleSign;
    config.voltage_sign = config::kGm6020VoltageSign;
    config.gear_ratio = config::kGm6020GearRatio;
    config.position =
    {
        config::kGm6020PosKp,
        config::kGm6020PosKi,
        config::kGm6020PosKd,
        config::kGm6020PosIntegralMax,
        config::kGm6020PosIntegralSeparate,
        config::kGm6020PosSeparationDeg
    };
    config.speed =
    {
        config::kGm6020SpeedKp,
        config::kGm6020SpeedKi,
        config::kGm6020SpeedKd,
        config::kGm6020SpeedIntegralMax,
        config::kGm6020SpeedIntegralSeparate,
        config::kGm6020SpeedSeparationRpm
    };
    config.control_dt_s = config::kControlDtS;
    config.max_speed_rpm = config::kGm6020MaxSpeedRpm;
    config.voltage_limit_raw = config::kGm6020VoltageLimitRaw;
    config.online_timeout_ms = config::kGm6020OnlineTimeoutMs;
    return config;
}

constexpr SetpointRampConfig kRampConfig = makeRampConfig();
constexpr Gm6020::Config kGm6020Config = makeGm6020Config();
constexpr Vesc::Config kVescConfig = makeVescConfig();

#if !WHEEL_BUILD_DEBUG
constexpr MasterCan::Config makeMasterConfig()
{
    MasterCan::Config config = {};
    config.wheel_id = config::kMasterWheelId;
    config.link_timeout_ms = config::kMasterLinkTimeoutMs;
    return config;
}

constexpr MasterCan::Config kMasterConfig = makeMasterConfig();
#else
constexpr WheelDebugConfig makeDebugConfig()
{
    WheelDebugConfig config = {};
    config.target_angle_deg = config::kDebugDefaultAngleDeg;
    config.direction_erpm = config::kDebugDefaultDirectionErpm;
    config.target_speed_rpm = config::kDebugDefaultSpeedRpm;
    config.target_voltage_raw = config::kDebugDefaultVoltageRaw;
    config.target_erpm = config::kDebugDefaultTargetErpm;
    config.erpm_limit = config::kDebugDefaultErpmLimit;
    config.use_ramp = config::kDebugDefaultUseRamp;
    config.wave_amplitude_deg = config::kDebugDefaultAmplitudeDeg;
    config.wave_period_ms = config::kDebugDefaultPeriodMs;
    config.min_period_ms = config::kDebugMinPeriodMs;
    config.max_amplitude_deg = config::kDebugMaxAmplitudeDeg;
    return config;
}

constexpr WheelDebugConfig kDebugConfig = makeDebugConfig();
#endif

bool isValidPidConfig(const PidConfig &pid)
{
    return std::isfinite(pid.kp) &&
           std::isfinite(pid.ki) &&
           std::isfinite(pid.kd) &&
           (pid.kp >= 0.0f) &&
           (pid.ki >= 0.0f) &&
           (pid.kd >= 0.0f) &&
           std::isfinite(pid.integral_max) &&
           (pid.integral_max >= 0.0f) &&
           (pid.integral_separation <= 1U) &&
           std::isfinite(pid.separation_threshold) &&
           (pid.separation_threshold >= 0.0f);
}

bool isValidConfig()
{
    if (!std::isfinite(config::kControlDtS) ||
        (config::kControlDtS <= 0.0f) ||
        !std::isfinite(config::kRampMaxSpeedDegS) ||
        (config::kRampMaxSpeedDegS <= 0.0f) ||
        !std::isfinite(config::kRampMaxAccelDegS2) ||
        (config::kRampMaxAccelDegS2 <= 0.0f) ||
        (config::kSelfTestMs == 0U) ||
        (config::kTelemetryMs == 0U) ||
        (config::kLedUpdateMs == 0U) ||
        (config::kDebugHeartbeatMs == 0U))
    {
        return false;
    }

    if ((kGm6020Config.id < 1U) ||
        (kGm6020Config.id > 7U) ||
        (kGm6020Config.zero_raw > 0x1FFFU) ||
        ((kGm6020Config.angle_sign != 1) &&
         (kGm6020Config.angle_sign != -1)) ||
        ((kGm6020Config.voltage_sign != 1) &&
         (kGm6020Config.voltage_sign != -1)) ||
        !std::isfinite(kGm6020Config.gear_ratio) ||
        (kGm6020Config.gear_ratio <= 0.0f) ||
        !std::isfinite(kGm6020Config.max_speed_rpm) ||
        (kGm6020Config.max_speed_rpm <= 0.0f) ||
        (kGm6020Config.voltage_limit_raw <= 0) ||
        (kGm6020Config.voltage_limit_raw >
         Gm6020::kMaxVoltageRaw) ||
        (kGm6020Config.online_timeout_ms == 0U) ||
        !isValidPidConfig(kGm6020Config.position) ||
        !isValidPidConfig(kGm6020Config.speed))
    {
        return false;
    }

    if ((kVescConfig.controller_id > 0xFFU) ||
        (kVescConfig.online_timeout_ms == 0U) ||
        !std::isfinite(config::kMasterAngleSign) ||
        (std::fabs(config::kMasterAngleSign) != 1.0f) ||
        !std::isfinite(config::kMasterAngleOffsetDeg))
    {
        return false;
    }

#if WHEEL_BUILD_DEBUG
    if ((kDebugConfig.use_ramp > 1U) ||
        (kDebugConfig.erpm_limit < 0) ||
        (kDebugConfig.min_period_ms == 0U) ||
        (kDebugConfig.wave_period_ms <
         kDebugConfig.min_period_ms) ||
        !std::isfinite(kDebugConfig.max_amplitude_deg) ||
        (kDebugConfig.max_amplitude_deg <= 0.0f) ||
        !std::isfinite(kDebugConfig.wave_amplitude_deg) ||
        (std::fabs(kDebugConfig.wave_amplitude_deg) >
         kDebugConfig.max_amplitude_deg) ||
        !std::isfinite(kDebugConfig.target_angle_deg) ||
        !std::isfinite(kDebugConfig.target_speed_rpm) ||
        (std::fabs(kDebugConfig.target_speed_rpm) >
         kGm6020Config.max_speed_rpm) ||
        (kDebugConfig.target_voltage_raw >
         kGm6020Config.voltage_limit_raw) ||
        (kDebugConfig.target_voltage_raw <
         -kGm6020Config.voltage_limit_raw))
    {
        return false;
    }
#else
    if ((kMasterConfig.wheel_id < 1U) ||
        (kMasterConfig.wheel_id > 4U) ||
        (kMasterConfig.link_timeout_ms == 0U))
    {
        return false;
    }
#endif

    return true;
}

uint32_t elapsedMs(uint32_t now_ms, uint32_t timestamp_ms)
{
    return static_cast<uint32_t>(now_ms - timestamp_ms);
}

} // namespace

void WheelApp::init(CAN_HandleTypeDef *local_can,
                    CAN_HandleTypeDef *master_can,
                    TIM_HandleTypeDef *timer,
                    UART_HandleTypeDef *uart)
{
    if ((local_can == nullptr) ||
        (timer == nullptr) ||
        (uart == nullptr) ||
        !isValidConfig())
    {
        Error_Handler();
        return;
    }

#if !WHEEL_BUILD_DEBUG
    if (master_can == nullptr)
    {
        Error_Handler();
        return;
    }
    hcan2_ = master_can;
#else
    (void)master_can;
#endif

    hcan1 = local_can;
    htim6 = timer;
    huart1 = uart;
    ms = 0U;
    gm6020_online = 0U;
    vesc_online = 0U;
    wheel_active = 0U;
    need_fresh_control = 1U;
    angle_reversed = 0U;
    mapped_target_deg = 0.0f;
    final_target_angle_deg = 0.0f;
    final_target_erpm = 0;

    gm6020.init(hcan1, kGm6020Config);
    vesc.init(hcan1, kVescConfig);
    target_ramp.init(0.0f, kRampConfig);

#if WHEEL_BUILD_DEBUG
    WheelDebug::init(g_wheel_debug, debug_runtime, kDebugConfig);
#else
    control_pending = 0U;
    master.init(kMasterConfig);
#endif

    telemetry_.reset();
    leds_.reset();
    HAL_NVIC_SetPriority(USART1_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void WheelApp::start()
{
    if ((hcan1 == nullptr) || (htim6 == nullptr))
    {
        Error_Handler();
        return;
    }
    if (bsp_can::startAcceptAll(hcan1) != HAL_OK)
    {
        Error_Handler();
        return;
    }

#if !WHEEL_BUILD_DEBUG
    if (bsp_can::startMasterFilter(
            hcan2_,
            master.control_id,
            master.heartbeat_id) != HAL_OK)
    {
        Error_Handler();
        return;
    }
#endif

    if (HAL_TIM_Base_Start_IT(htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

void WheelApp::loop()
{
    leds_.poll(*this, ms);
    telemetry_.poll(*this, ms);
}

void WheelApp::updateOnlineState()
{
    gm6020_online =
        ((gm6020.is_online != 0U) &&
         (elapsedMs(ms, gm6020.last_rx_ms) <=
          gm6020.online_timeout_ms))
            ? 1U
            : 0U;
    vesc_online =
        ((vesc.is_online != 0U) &&
         (elapsedMs(ms, vesc.last_rx_ms) <=
          vesc.online_timeout_ms))
            ? 1U
            : 0U;
}

void WheelApp::disableActuators()
{
    if (gm6020.is_enabled != 0U)
    {
        gm6020.disable();
    }
    if (vesc.is_enabled != 0U)
    {
        vesc.disable();
    }
    wheel_active = 0U;
}

void WheelApp::disableWheel()
{
    disableActuators();
    need_fresh_control = 1U;
}

void WheelApp::controlStep()
{
#if WHEEL_BUILD_DEBUG
    debug_controller_.step(*this);
#else
    production_controller_.step(*this);
#endif
}

uint32_t WheelApp::faultFlags() const
{
    uint32_t flags = 0U;

#if WHEEL_BUILD_DEBUG
    const WheelDebugMode mode = g_wheel_debug.mode;
    const bool uses_gm =
        WheelDebug::usesGmPosition(mode) ||
        WheelDebug::usesGmSpeed(mode) ||
        WheelDebug::usesGmVoltage(mode);
    if (uses_gm && (gm6020_online == 0U))
    {
        flags |= config::kFaultGm6020Offline;
    }
    if (WheelDebug::usesVesc(mode) && (vesc_online == 0U))
    {
        flags |= config::kFaultVescOffline;
    }
    if (uses_gm && (gm6020.last_tx_status != HAL_OK))
    {
        flags |= config::kFaultGm6020Tx;
    }
    if (WheelDebug::usesVesc(mode) &&
        (vesc.last_tx_status != HAL_OK))
    {
        flags |= config::kFaultVescTx;
    }
#else
    if (master.link_valid == 0U)
    {
        flags |= config::kFaultMasterLink;
    }
    if (master.control_seen == 0U)
    {
        flags |= config::kFaultNoControl;
    }
    if (gm6020_online == 0U)
    {
        flags |= config::kFaultGm6020Offline;
    }
    if (vesc_online == 0U)
    {
        flags |= config::kFaultVescOffline;
    }
    if (need_fresh_control != 0U)
    {
        flags |= config::kFaultNeedFresh;
    }
    if (gm6020.last_tx_status != HAL_OK)
    {
        flags |= config::kFaultGm6020Tx;
    }
    if (vesc.last_tx_status != HAL_OK)
    {
        flags |= config::kFaultVescTx;
    }
#endif

    return flags;
}

void WheelApp::onCanRx(CAN_HandleTypeDef *hcan)
{
    if (hcan == nullptr)
    {
        return;
    }

    CAN_RxHeaderTypeDef header = {};
    uint8_t data[8] = {};

    if (hcan == hcan1)
    {
        while (HAL_CAN_GetRxFifoFillLevel(
                   hcan,
                   CAN_RX_FIFO0) > 0U)
        {
            if (HAL_CAN_GetRxMessage(
                    hcan,
                    CAN_RX_FIFO0,
                    &header,
                    data) != HAL_OK)
            {
                break;
            }
            gm6020.onRx(&header, data, ms);
            vesc.onRx(&header, data, ms);
        }
        return;
    }

#if !WHEEL_BUILD_DEBUG
    if (hcan == hcan2_)
    {
        while (HAL_CAN_GetRxFifoFillLevel(
                   hcan,
                   CAN_RX_FIFO0) > 0U)
        {
            if (HAL_CAN_GetRxMessage(
                    hcan,
                    CAN_RX_FIFO0,
                    &header,
                    data) != HAL_OK)
            {
                break;
            }

            const bool valid_control =
                (header.IDE == CAN_ID_STD) &&
                (header.RTR == CAN_RTR_DATA) &&
                (header.DLC == 8U) &&
                (header.StdId == master.control_id);
            master.onRx(&header, data, ms);
            if (valid_control)
            {
                control_pending = 1U;
            }
        }
    }
#endif
}

void WheelApp::onTimerElapsed(TIM_HandleTypeDef *htim)
{
    if ((htim == nullptr) ||
        (htim6 == nullptr) ||
        (htim->Instance != htim6->Instance))
    {
        return;
    }
    ++ms;
    controlStep();
}

void WheelApp::onUartTxComplete(
    UART_HandleTypeDef *huart)
{
    if (huart == huart1)
    {
        telemetry_.onTxComplete();
    }
}

} // namespace steering

steering::WheelApp g_app;
