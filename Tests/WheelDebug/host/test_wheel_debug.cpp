#include <cmath>
#include <cstdint>
#include <cstdio>

#include "WheelDebug.hpp"

static uint32_t g_failures;

static void InitDebug(steering::WheelDebugInput &debug,
                      steering::WheelDebugRuntime &runtime,
                      steering::SetpointRamp &ramp)
{
    steering::WheelDebugConfig config = {};
    config.target_angle_deg = 0.0f;
    config.direction_erpm = 0;
    config.target_speed_rpm = 50.0f;
    config.target_voltage_raw = 0;
    config.target_erpm = 0;
    config.erpm_limit = 1000;
    config.use_ramp = 1U;
    config.wave_amplitude_deg = 30.0f;
    config.wave_period_ms = 4000U;
    config.min_period_ms = 200U;
    config.max_amplitude_deg = 180.0f;
    steering::SetpointRampConfig ramp_config = {};
    ramp_config.max_speed_deg_s = 790.0f;
    ramp_config.max_accel_deg_s2 = 4000.0f;

    steering::WheelDebug::init(debug, runtime, config);
    ramp.init(0.0f, ramp_config);
}

static void Check(bool condition, const char *message)
{
    if (!condition)
    {
        std::printf("FAIL: %s\n", message);
        g_failures++;
    }
}

static void Test_InitAndModeChange(void)
{
    steering::WheelDebugInput debug;
    steering::WheelDebugRuntime runtime;
    steering::SetpointRamp ramp;

    InitDebug(debug, runtime, ramp);
    Check(debug.mode == steering::WheelDebugMode::Disabled,
          "debug defaults to disabled");
    Check(debug.enable == 0U, "debug defaults to disarmed");
    Check(debug.use_ramp == 1U, "debug defaults to ramp enabled");
    Check(runtime.min_period_ms == 200U,
          "debug minimum period injected from config");
    Check(std::fabs(runtime.max_amplitude_deg - 180.0f) < 0.001f,
          "debug amplitude limit injected from config");

    debug.mode = steering::WheelDebugMode::GmPositionStep;
    debug.enable = 1U;
    steering::WheelDebug::update(
        debug, runtime, ramp, 0.0f, 0.001f);
    Check(runtime.mode_changed == 1U,
          "mode change reported");
    Check(debug.enable == 0U,
          "mode change clears debug enable");

    debug.enable = 1U;
    steering::WheelDebug::update(
        debug, runtime, ramp, 0.0f, 0.001f);
    Check(runtime.mode_changed == 0U,
          "stable mode not reported as changed");

    debug.mode = static_cast<steering::WheelDebugMode>(99U);
    steering::WheelDebug::update(
        debug, runtime, ramp, 0.0f, 0.001f);
    Check(debug.mode == steering::WheelDebugMode::Disabled,
          "invalid mode falls back to disabled");
    Check(debug.enable == 0U,
          "invalid mode clears enable");
}

static void Test_PositionOptimizationAndRamp(void)
{
    steering::WheelDebugInput debug;
    steering::WheelDebugRuntime runtime;
    steering::SetpointRamp ramp;

    InitDebug(debug, runtime, ramp);
    debug.mode = steering::WheelDebugMode::GmPositionStep;
    debug.target_angle_deg = 100.0f;
    debug.direction_erpm = 100;
    debug.use_ramp = 0U;
    steering::WheelDebug::update(
        debug, runtime, ramp, 0.0f, 0.001f);
    debug.enable = 1U;
    steering::WheelDebug::update(
        debug, runtime, ramp, 0.0f, 0.001f);

    Check(runtime.angle_result.reversed == 1U,
          "positive direction ERPM selects reverse candidate");
    Check(std::fabs(runtime.ramp_output_angle_deg - 280.0f) < 0.01f,
          "no-ramp position uses optimized target");

    debug.direction_erpm = 0;
    steering::WheelDebug::update(
        debug, runtime, ramp, 0.0f, 0.001f);
    Check(runtime.angle_result.reversed == 1U,
          "zero direction ERPM still selects closer candidate");
    Check(std::fabs(runtime.ramp_output_angle_deg - 280.0f) < 0.01f,
          "zero direction ERPM uses optimized target");

    debug.use_ramp = 1U;
    steering::WheelDebug::update(
        debug, runtime, ramp, 0.0f, 0.001f);
    Check(debug.enable == 1U,
          "ramp toggle preserves enable");
    Check((std::fabs(runtime.ramp_output_angle_deg) < 1.0f) ||
          (std::fabs(runtime.ramp_output_angle_deg - 360.0f) < 1.0f),
          "ramp toggle restarts from current angle");
}

static void Test_SineLimits(void)
{
    steering::WheelDebugInput debug;
    steering::WheelDebugRuntime runtime;
    steering::SetpointRamp ramp;
    uint32_t i;

    InitDebug(debug, runtime, ramp);
    debug.mode = steering::WheelDebugMode::GmPositionSine;
    debug.use_ramp = 0U;
    debug.wave_amplitude_deg = 500.0f;
    debug.wave_period_ms = 1U;
    steering::WheelDebug::update(
        debug, runtime, ramp, 10.0f, 0.001f);
    debug.enable = 1U;
    steering::WheelDebug::update(
        debug, runtime, ramp, 10.0f, 0.001f);

    Check(std::fabs(runtime.source_angle_deg - 10.0f) < 0.01f,
          "sine starts at current position");

    for (i = 0U; i < 100U; i++)
    {
        steering::WheelDebug::update(
            debug, runtime, ramp, 10.0f, 0.001f);
        Check(runtime.source_angle_deg >= 0.0f,
              "sine angle normalized low bound");
        Check(runtime.source_angle_deg < 360.0f,
              "sine angle normalized high bound");
    }

    Check(std::fabs(debug.wave_amplitude_deg - 500.0f) < 0.001f,
          "watch amplitude value not overwritten");
}

int main()
{
    Test_InitAndModeChange();
    Test_PositionOptimizationAndRamp();
    Test_SineLimits();

    if (g_failures != 0U)
    {
        std::printf("WheelDebug host tests: %u failure(s)\n",
                    static_cast<unsigned>(g_failures));
        return 1;
    }

    std::printf("WheelDebug host tests: all passed\n");
    return 0;
}
