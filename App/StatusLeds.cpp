#include "StatusLeds.hpp"

#include "AppConfig.hpp"
#include "WheelApp.hpp"

namespace steering
{

namespace
{

uint32_t elapsedMs(uint32_t now_ms, uint32_t timestamp_ms)
{
    return static_cast<uint32_t>(now_ms - timestamp_ms);
}

} // namespace

void StatusLeds::reset()
{
    last_update_ms_ = 0U;
}

void StatusLeds::poll(WheelApp &app, uint32_t now_ms)
{
    if (elapsedMs(now_ms, last_update_ms_) <
        config::LedUpdateMs)
    {
        return;
    }
    last_update_ms_ = now_ms;
    update(app);
}

void StatusLeds::update(WheelApp &app)
{
    if (app.ms < config::SelfTestMs)
    {
        HAL_GPIO_WritePin(
            GPIOB,
            config::LedMasterPin |
                config::LedVescPin |
                config::LedEnablePin |
                config::LedGm6020Pin,
            GPIO_PIN_RESET);
        return;
    }

    uint8_t enabled = 0U;

#if WHEEL_BUILD_DEBUG
    const uint8_t heartbeat =
        (((app.ms / config::DebugHeartbeatMs) % 2U) == 0U)
            ? 1U
            : 0U;
    const WheelDebugMode mode = g_wheel_debug.mode;
    if (WheelDebug::usesVesc(mode))
    {
        enabled = app.vesc.is_enabled;
    }
    else if (WheelDebug::usesGmPosition(mode) ||
             WheelDebug::usesGmSpeed(mode) ||
             WheelDebug::usesGmVoltage(mode))
    {
        enabled = app.gm6020.is_enabled;
    }
    HAL_GPIO_WritePin(
        GPIOB,
        config::LedMasterPin,
        (heartbeat != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    enabled =
        ((app.wheel_active != 0U) &&
         (app.gm6020.is_enabled != 0U) &&
         (app.vesc.is_enabled != 0U))
            ? 1U
            : 0U;
    HAL_GPIO_WritePin(
        GPIOB,
        config::LedMasterPin,
        (app.master.link_valid != 0U)
            ? GPIO_PIN_RESET
            : GPIO_PIN_SET);
#endif

    HAL_GPIO_WritePin(
        GPIOB,
        config::LedVescPin,
        (app.vesc_online != 0U)
            ? GPIO_PIN_RESET
            : GPIO_PIN_SET);
    HAL_GPIO_WritePin(
        GPIOB,
        config::LedEnablePin,
        (enabled != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(
        GPIOB,
        config::LedGm6020Pin,
        (app.gm6020_online != 0U)
            ? GPIO_PIN_RESET
            : GPIO_PIN_SET);
}

} // namespace steering
