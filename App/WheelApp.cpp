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
    config.max_speed_deg_s = config::RampMaxSpeedDegS;
    config.max_accel_deg_s2 = config::RampMaxAccelDegS2;
    return config;
}

constexpr Vesc::Config makeVescConfig()
{
    Vesc::Config config = {};
    config.controller_id = config::VescControllerId;
    config.online_timeout_ms = config::VescOnlineTimeoutMs;
    return config;
}

constexpr Gm6020::Config makeGm6020Config()
{
    Gm6020::Config config = {};
    config.id = config::Gm6020Id;
    config.zero_raw = config::Gm6020ZeroRaw;
    config.angle_sign = config::Gm6020AngleSign;
    config.voltage_sign = config::Gm6020VoltageSign;
    config.gear_ratio = config::Gm6020GearRatio;
    config.position =
    {
        config::Gm6020PosKp,
        config::Gm6020PosKi,
        config::Gm6020PosKd,
        config::Gm6020PosIntegralMax,
        config::Gm6020PosIntegralSeparate,
        config::Gm6020PosSeparationDeg
    };
    config.speed =
    {
        config::Gm6020SpeedKp,
        config::Gm6020SpeedKi,
        config::Gm6020SpeedKd,
        config::Gm6020SpeedIntegralMax,
        config::Gm6020SpeedIntegralSeparate,
        config::Gm6020SpeedSeparationRpm
    };
    config.control_dt_s = config::ControlDtS;
    config.max_speed_rpm = config::Gm6020MaxSpeedRpm;
    config.voltage_limit_raw = config::Gm6020VoltageLimitRaw;
    config.online_timeout_ms = config::Gm6020OnlineTimeoutMs;
    return config;
}

constexpr SetpointRampConfig RampConfig = makeRampConfig();
constexpr Gm6020::Config Gm6020Config = makeGm6020Config();
constexpr Vesc::Config VescConfig = makeVescConfig();

#if !WHEEL_BUILD_DEBUG
constexpr MasterCan::Config makeMasterConfig()
{
    MasterCan::Config config = {};
    config.wheel_id = config::MasterWheelId;
    config.link_timeout_ms = config::MasterLinkTimeoutMs;
    return config;
}

constexpr MasterCan::Config MasterConfig = makeMasterConfig();
#else
constexpr WheelDebugConfig makeDebugConfig()
{
    WheelDebugConfig config = {};
    config.target_angle_deg = config::DebugDefaultAngleDeg;
    config.direction_erpm = config::DebugDefaultDirectionErpm;
    config.target_speed_rpm = config::DebugDefaultSpeedRpm;
    config.target_voltage_raw = config::DebugDefaultVoltageRaw;
    config.target_erpm = config::DebugDefaultTargetErpm;
    config.erpm_limit = config::DebugDefaultErpmLimit;
    config.use_ramp = config::DebugDefaultUseRamp;
    config.wave_amplitude_deg = config::DebugDefaultAmplitudeDeg;
    config.wave_period_ms = config::DebugDefaultPeriodMs;
    config.min_period_ms = config::DebugMinPeriodMs;
    config.max_amplitude_deg = config::DebugMaxAmplitudeDeg;
    return config;
}

constexpr WheelDebugConfig DebugConfig = makeDebugConfig();
#endif

bool isValidPidConfig(const Pid::Config &pid)
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
    if (!std::isfinite(config::ControlDtS) ||
        (config::ControlDtS <= 0.0f) ||
        !std::isfinite(config::RampMaxSpeedDegS) ||
        (config::RampMaxSpeedDegS <= 0.0f) ||
        !std::isfinite(config::RampMaxAccelDegS2) ||
        (config::RampMaxAccelDegS2 <= 0.0f) ||
        (config::SelfTestMs == 0U) ||
        (config::TelemetryMs == 0U) ||
        (config::LedUpdateMs == 0U) ||
        (config::DebugHeartbeatMs == 0U))
    {
        return false;
    }

    if ((Gm6020Config.id < 1U) ||
        (Gm6020Config.id > 7U) ||
        (Gm6020Config.zero_raw > 0x1FFFU) ||
        ((Gm6020Config.angle_sign != 1) &&
         (Gm6020Config.angle_sign != -1)) ||
        ((Gm6020Config.voltage_sign != 1) &&
         (Gm6020Config.voltage_sign != -1)) ||
        !std::isfinite(Gm6020Config.gear_ratio) ||
        (Gm6020Config.gear_ratio <= 0.0f) ||
        !std::isfinite(Gm6020Config.max_speed_rpm) ||
        (Gm6020Config.max_speed_rpm <= 0.0f) ||
        (Gm6020Config.voltage_limit_raw <= 0) ||
        (Gm6020Config.voltage_limit_raw >
         Gm6020::MaxVoltageRaw) ||
        (Gm6020Config.online_timeout_ms == 0U) ||
        !isValidPidConfig(Gm6020Config.position) ||
        !isValidPidConfig(Gm6020Config.speed))
    {
        return false;
    }

    if ((VescConfig.controller_id > 0xFFU) ||
        (VescConfig.online_timeout_ms == 0U) ||
        !std::isfinite(config::MasterAngleSign) ||
        (std::fabs(config::MasterAngleSign) != 1.0f) ||
        !std::isfinite(config::MasterAngleOffsetDeg))
    {
        return false;
    }

#if WHEEL_BUILD_DEBUG
    if ((DebugConfig.use_ramp > 1U) ||
        (DebugConfig.erpm_limit < 0) ||
        (DebugConfig.min_period_ms == 0U) ||
        (DebugConfig.wave_period_ms <
         DebugConfig.min_period_ms) ||
        !std::isfinite(DebugConfig.max_amplitude_deg) ||
        (DebugConfig.max_amplitude_deg <= 0.0f) ||
        !std::isfinite(DebugConfig.wave_amplitude_deg) ||
        (std::fabs(DebugConfig.wave_amplitude_deg) >
         DebugConfig.max_amplitude_deg) ||
        !std::isfinite(DebugConfig.target_angle_deg) ||
        !std::isfinite(DebugConfig.target_speed_rpm) ||
        (std::fabs(DebugConfig.target_speed_rpm) >
         Gm6020Config.max_speed_rpm) ||
        (DebugConfig.target_voltage_raw >
         Gm6020Config.voltage_limit_raw) ||
        (DebugConfig.target_voltage_raw <
         -Gm6020Config.voltage_limit_raw))
    {
        return false;
    }
#else
    if ((MasterConfig.wheel_id < 1U) ||
        (MasterConfig.wheel_id > 4U) ||
        (MasterConfig.link_timeout_ms == 0U))
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

    gm6020.init(hcan1, Gm6020Config);
    vesc.init(hcan1, VescConfig);
    target_ramp.init(0.0f, RampConfig);

#if WHEEL_BUILD_DEBUG
    WheelDebug::init(g_wheel_debug, debug_runtime, DebugConfig);
#else
    control_pending = 0U;
    master.init(MasterConfig);
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
        flags |= config::FaultGm6020Offline;
    }
    if (WheelDebug::usesVesc(mode) && (vesc_online == 0U))
    {
        flags |= config::FaultVescOffline;
    }
    if (uses_gm && (gm6020.last_tx_status != HAL_OK))
    {
        flags |= config::FaultGm6020Tx;
    }
    if (WheelDebug::usesVesc(mode) &&
        (vesc.last_tx_status != HAL_OK))
    {
        flags |= config::FaultVescTx;
    }
#else
    if (master.link_valid == 0U)
    {
        flags |= config::FaultMasterLink;
    }
    if (master.control_seen == 0U)
    {
        flags |= config::FaultNoControl;
    }
    if (gm6020_online == 0U)
    {
        flags |= config::FaultGm6020Offline;
    }
    if (vesc_online == 0U)
    {
        flags |= config::FaultVescOffline;
    }
    if (need_fresh_control != 0U)
    {
        flags |= config::FaultNeedFresh;
    }
    if (gm6020.last_tx_status != HAL_OK)
    {
        flags |= config::FaultGm6020Tx;
    }
    if (vesc.last_tx_status != HAL_OK)
    {
        flags |= config::FaultVescTx;
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
