#include "WheelDebug.hpp"

#include <cmath>

namespace steering
{

namespace
{

constexpr float kTwoPi = 6.283185307179586f;

} // namespace

extern "C"
{
steering::WheelDebugInput g_wheel_debug = {};
}

bool WheelDebug::isValidMode(WheelDebugMode mode)
{
    return static_cast<uint8_t>(mode) <=
           static_cast<uint8_t>(WheelDebugMode::VescSpeed);
}

bool WheelDebug::isPositionMode(WheelDebugMode mode)
{
    return (mode == WheelDebugMode::GmPositionStep) ||
           (mode == WheelDebugMode::GmPositionSine);
}

float WheelDebug::clampAbs(float value, float limit)
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

float WheelDebug::wrap360(float angle_deg)
{
    angle_deg = std::fmod(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

void WheelDebug::reset(WheelDebugRuntime &runtime)
{
    runtime.angle_result.target_angle_deg = 0.0f;
    runtime.angle_result.target_erpm = 0;
    runtime.angle_result.reversed = 0U;
    runtime.source_angle_deg = 0.0f;
    runtime.ramp_output_angle_deg = 0.0f;
    runtime.sine_origin_deg = 0.0f;
    runtime.sine_phase = 0.0f;
    runtime.initialized = 0U;
    runtime.ramp_active = 0U;
}

void WheelDebug::init(WheelDebugInput &debug,
                      WheelDebugRuntime &runtime,
                      const WheelDebugConfig &config)
{
    debug.mode = WheelDebugMode::Disabled;
    debug.enable = 0U;
    debug.target_angle_deg = config.target_angle_deg;
    debug.direction_erpm = config.direction_erpm;
    debug.target_speed_rpm = config.target_speed_rpm;
    debug.target_voltage_raw =
        static_cast<int16_t>(config.target_voltage_raw);
    debug.target_erpm = config.target_erpm;
    debug.erpm_limit = config.erpm_limit;
    debug.use_ramp = config.use_ramp;
    debug.wave_amplitude_deg = config.wave_amplitude_deg;
    debug.wave_period_ms = config.wave_period_ms;

    reset(runtime);
    runtime.min_period_ms = config.min_period_ms;
    runtime.max_amplitude_deg =
        std::fabs(config.max_amplitude_deg);
    runtime.previous_mode = WheelDebugMode::Disabled;
    runtime.mode_changed = 0U;
}

void WheelDebug::update(WheelDebugInput &debug,
                        WheelDebugRuntime &runtime,
                        SetpointRamp &ramp,
                        float current_angle_deg,
                        float dt_s)
{
    if (dt_s <= 0.0f)
    {
        return;
    }

    WheelDebugMode mode = debug.mode;
    if (!isValidMode(mode))
    {
        mode = WheelDebugMode::Disabled;
        debug.mode = mode;
    }

    runtime.mode_changed = 0U;
    if (mode != runtime.previous_mode)
    {
        debug.enable = 0U;
        reset(runtime);
        runtime.previous_mode = mode;
        runtime.mode_changed = 1U;
    }

    if (!isPositionMode(mode) || (debug.enable == 0U))
    {
        runtime.initialized = 0U;
        return;
    }

    const uint8_t use_ramp = (debug.use_ramp != 0U) ? 1U : 0U;
    if ((runtime.initialized == 0U) ||
        (use_ramp != runtime.ramp_active))
    {
        runtime.sine_origin_deg = wrap360(current_angle_deg);
        runtime.sine_phase = 0.0f;
        runtime.ramp_active = use_ramp;
        runtime.initialized = 1U;
        ramp.reset(runtime.sine_origin_deg);
        runtime.ramp_output_angle_deg =
            runtime.sine_origin_deg;
    }

    float desired_angle_deg;
    if (mode == WheelDebugMode::GmPositionSine)
    {
        float amplitude_deg =
            std::fabs(debug.wave_amplitude_deg);
        amplitude_deg = clampAbs(
            amplitude_deg,
            runtime.max_amplitude_deg);

        uint32_t period_ms = debug.wave_period_ms;
        if (period_ms < runtime.min_period_ms)
        {
            period_ms = runtime.min_period_ms;
        }

        desired_angle_deg =
            runtime.sine_origin_deg +
            (amplitude_deg *
             std::sin(kTwoPi * runtime.sine_phase));
        runtime.sine_phase +=
            dt_s / (static_cast<float>(period_ms) * 0.001f);
        if (runtime.sine_phase >= 1.0f)
        {
            runtime.sine_phase -= 1.0f;
        }
    }
    else
    {
        desired_angle_deg = debug.target_angle_deg;
    }

    runtime.source_angle_deg = wrap360(desired_angle_deg);
    runtime.angle_result = angle_opt::select(
        current_angle_deg,
        runtime.source_angle_deg,
        debug.direction_erpm);

    if (runtime.ramp_active != 0U)
    {
        ramp.setDesired(runtime.angle_result.target_angle_deg);
        ramp.update(dt_s);
        runtime.ramp_output_angle_deg = ramp.output();
    }
    else
    {
        runtime.ramp_output_angle_deg =
            runtime.angle_result.target_angle_deg;
    }
}

bool WheelDebug::usesGmPosition(WheelDebugMode mode)
{
    return isPositionMode(mode);
}

bool WheelDebug::usesGmSpeed(WheelDebugMode mode)
{
    return mode == WheelDebugMode::GmSpeed;
}

bool WheelDebug::usesGmVoltage(WheelDebugMode mode)
{
    return mode == WheelDebugMode::GmVoltage;
}

bool WheelDebug::usesVesc(WheelDebugMode mode)
{
    return mode == WheelDebugMode::VescSpeed;
}

} // namespace steering
