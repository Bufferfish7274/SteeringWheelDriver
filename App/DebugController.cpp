#include "DebugController.hpp"

#include <cmath>

#include "AppConfig.hpp"
#include "WheelApp.hpp"

#if WHEEL_BUILD_DEBUG

namespace steering
{

bool DebugController::usesGm(WheelDebugMode mode)
{
    return WheelDebug::usesGmPosition(mode) ||
           WheelDebug::usesGmSpeed(mode) ||
           WheelDebug::usesGmVoltage(mode);
}

float DebugController::clampAbs(float value, float limit)
{
    if (limit <= 0.0f)
    {
        return 0.0f;
    }
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void DebugController::clearEnable(WheelApp &app)
{
    g_wheel_debug.enable = 0U;
    WheelDebug::reset(app.debug_runtime);
    app.disableActuators();
    app.angle_reversed = 0U;
    app.mapped_target_deg = 0.0f;
    app.final_target_angle_deg = 0.0f;
    app.final_target_erpm = 0;
}

void DebugController::step(WheelApp &app)
{
    app.updateOnlineState();
    WheelDebug::update(
        g_wheel_debug,
        app.debug_runtime,
        app.target_ramp,
        app.gm6020.angle_deg,
        config::kControlDtS);

    const uint8_t debug_enable = g_wheel_debug.enable;
    if (app.debug_runtime.mode_changed != 0U)
    {
        clearEnable(app);
    }

    const WheelDebugMode mode = app.debug_runtime.previous_mode;
    if (app.ms < config::kSelfTestMs)
    {
        clearEnable(app);
        app.gm6020.controlStep(app.ms);
        app.vesc.controlStep(app.ms);
        return;
    }

    if (usesGm(mode) && (app.gm6020_online == 0U))
    {
        clearEnable(app);
    }
    if (WheelDebug::usesVesc(mode) &&
        (app.vesc_online == 0U))
    {
        clearEnable(app);
    }

    switch (mode)
    {
        case WheelDebugMode::GmPositionStep:
        case WheelDebugMode::GmPositionSine:
            app.gm6020.setControlMode(Gm6020::ControlMode::Position);
            app.vesc.setTargetErpm(0);
            if (app.vesc.is_enabled != 0U)
            {
                app.vesc.disable();
            }

            if ((debug_enable != 0U) &&
                (app.gm6020_online != 0U))
            {
                if (app.gm6020.is_enabled == 0U)
                {
                    app.gm6020.enable();
                }
                app.gm6020.setTargetAngle(
                    app.debug_runtime.ramp_output_angle_deg);
                app.mapped_target_deg =
                    app.debug_runtime.source_angle_deg;
                app.final_target_angle_deg =
                    app.debug_runtime.angle_result.target_angle_deg;
                app.final_target_erpm =
                    app.debug_runtime.angle_result.target_erpm;
                app.angle_reversed =
                    app.debug_runtime.angle_result.reversed;
                app.wheel_active = app.gm6020.is_enabled;
            }
            else
            {
                if (app.gm6020.is_enabled != 0U)
                {
                    app.gm6020.disable();
                }
                app.wheel_active = 0U;
            }
            break;

        case WheelDebugMode::GmSpeed:
            app.gm6020.setControlMode(Gm6020::ControlMode::Speed);
            app.vesc.setTargetErpm(0);
            if (app.vesc.is_enabled != 0U)
            {
                app.vesc.disable();
            }

            if ((debug_enable != 0U) &&
                (app.gm6020_online != 0U))
            {
                if (app.gm6020.is_enabled == 0U)
                {
                    app.gm6020.enable();
                }
                app.gm6020.setTargetSpeed(
                    g_wheel_debug.target_speed_rpm);
                app.final_target_erpm = 0;
                app.wheel_active = app.gm6020.is_enabled;
            }
            else
            {
                if (app.gm6020.is_enabled != 0U)
                {
                    app.gm6020.disable();
                }
                app.wheel_active = 0U;
            }
            break;

        case WheelDebugMode::GmVoltage:
            app.gm6020.setControlMode(Gm6020::ControlMode::Voltage);
            app.vesc.setTargetErpm(0);
            if (app.vesc.is_enabled != 0U)
            {
                app.vesc.disable();
            }

            if ((debug_enable != 0U) &&
                (app.gm6020_online != 0U))
            {
                if (app.gm6020.is_enabled == 0U)
                {
                    app.gm6020.enable();
                }
                app.gm6020.setTargetVoltage(
                    g_wheel_debug.target_voltage_raw);
                app.final_target_erpm = 0;
                app.wheel_active = app.gm6020.is_enabled;
            }
            else
            {
                if (app.gm6020.is_enabled != 0U)
                {
                    app.gm6020.disable();
                }
                app.wheel_active = 0U;
            }
            break;

        case WheelDebugMode::VescSpeed:
        {
            if (app.gm6020.is_enabled != 0U)
            {
                app.gm6020.disable();
            }

            int32_t erpm_limit = g_wheel_debug.erpm_limit;
            if (erpm_limit < 0)
            {
                erpm_limit = -erpm_limit;
            }

            if ((debug_enable != 0U) &&
                (app.vesc_online != 0U))
            {
                if (app.vesc.is_enabled == 0U)
                {
                    app.vesc.enable();
                }

                const int32_t target_erpm =
                    static_cast<int32_t>(clampAbs(
                        static_cast<float>(g_wheel_debug.target_erpm),
                        static_cast<float>(erpm_limit)));
                app.vesc.setTargetErpm(target_erpm);
                app.final_target_erpm = target_erpm;
                app.wheel_active = app.vesc.is_enabled;
            }
            else
            {
                app.vesc.setTargetErpm(0);
                if (app.vesc.is_enabled != 0U)
                {
                    app.vesc.disable();
                }
                app.wheel_active = 0U;
            }
            break;
        }

        case WheelDebugMode::Disabled:
        default:
            clearEnable(app);
            break;
    }

    app.gm6020.controlStep(app.ms);
    app.vesc.controlStep(app.ms);
}

} // namespace steering

#endif
