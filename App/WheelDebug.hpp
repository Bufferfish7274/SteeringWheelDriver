#ifndef WHEEL_DEBUG_HPP
#define WHEEL_DEBUG_HPP

#include <cstdint>

#include "AngleOpt.hpp"
#include "SetpointRamp.hpp"

namespace steering
{

enum class WheelDebugMode : uint8_t
{
    Disabled = 0U,
    GmPositionStep,
    GmPositionSine,
    GmSpeed,
    GmVoltage,
    VescSpeed
};

struct WheelDebugConfig
{
    float target_angle_deg;
    int32_t direction_erpm;
    float target_speed_rpm;
    int32_t target_voltage_raw;
    int32_t target_erpm;
    int32_t erpm_limit;
    uint8_t use_ramp;
    float wave_amplitude_deg;
    uint32_t wave_period_ms;
    uint32_t min_period_ms;
    float max_amplitude_deg;
};

struct WheelDebugInput
{
    volatile WheelDebugMode mode;
    volatile uint8_t enable;
    volatile float target_angle_deg;
    volatile int32_t direction_erpm;
    volatile float target_speed_rpm;
    volatile int16_t target_voltage_raw;
    volatile int32_t target_erpm;
    volatile int32_t erpm_limit;
    volatile uint8_t use_ramp;
    volatile float wave_amplitude_deg;
    volatile uint32_t wave_period_ms;
};

struct WheelDebugRuntime
{
    angle_opt::Result angle_result;
    WheelDebugMode previous_mode;
    float source_angle_deg;
    float ramp_output_angle_deg;
    float sine_origin_deg;
    float sine_phase;
    uint8_t initialized;
    uint8_t ramp_active;
    uint8_t mode_changed;
    uint32_t min_period_ms;
    float max_amplitude_deg;
};

class WheelDebug
{
public:
    static void init(WheelDebugInput &debug,
                     WheelDebugRuntime &runtime,
                     const WheelDebugConfig &config);
    static void reset(WheelDebugRuntime &runtime);
    static void update(WheelDebugInput &debug,
                       WheelDebugRuntime &runtime,
                       SetpointRamp &ramp,
                       float current_angle_deg,
                       float dt_s);

    static bool usesGmPosition(WheelDebugMode mode);
    static bool usesGmSpeed(WheelDebugMode mode);
    static bool usesGmVoltage(WheelDebugMode mode);
    static bool usesVesc(WheelDebugMode mode);

private:
    static bool isValidMode(WheelDebugMode mode);
    static bool isPositionMode(WheelDebugMode mode);
    static float clampAbs(float value, float limit);
    static float wrap360(float angle_deg);
};

} // namespace steering

extern "C" steering::WheelDebugInput g_wheel_debug;

#endif
