#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "WheelDebug.h"

static uint32_t g_failures;

static void InitDebug(WheelDebug_t *debug,
                      WheelDebugRuntime_t *runtime,
                      SetpointRamp_t *ramp)
{
    WheelDebug_Config_t config =
    {
        .target_angle_deg = 0.0f,
        .direction_erpm = 0,
        .target_speed_rpm = 50.0f,
        .target_voltage_raw = 0,
        .target_erpm = 0,
        .erpm_limit = 1000,
        .use_ramp = 1U,
        .wave_amplitude_deg = 30.0f,
        .wave_period_ms = 4000U,
        .min_period_ms = 200U,
        .max_amplitude_deg = 180.0f
    };
    SetpointRamp_Config_t ramp_config =
    {
        .max_speed_deg_s = 790.0f,
        .max_accel_deg_s2 = 4000.0f
    };

    WheelDebug_Init(debug, runtime, &config);
    SetpointRamp_Init(ramp, 0.0f, &ramp_config);
}

static void Check(int condition, const char *message)
{
    if (!condition)
    {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

static void Test_InitAndModeChange(void)
{
    WheelDebug_t debug;
    WheelDebugRuntime_t runtime;
    SetpointRamp_t ramp;

    InitDebug(&debug, &runtime, &ramp);
    Check(debug.mode == WHEEL_DEBUG_DISABLED,
          "debug defaults to disabled");
    Check(debug.enable == 0U, "debug defaults to disarmed");
    Check(debug.use_ramp == 1U, "debug defaults to ramp enabled");
    Check(runtime.min_period_ms == 200U,
          "debug minimum period injected from config");
    Check(fabsf(runtime.max_amplitude_deg - 180.0f) < 0.001f,
          "debug amplitude limit injected from config");

    debug.mode = WHEEL_DEBUG_GM_POSITION_STEP;
    debug.enable = 1U;
    WheelDebug_Update(&debug, &runtime, &ramp, 0.0f, 0.001f);
    Check(runtime.mode_changed == 1U,
          "mode change reported");
    Check(debug.enable == 0U,
          "mode change clears debug enable");

    debug.enable = 1U;
    WheelDebug_Update(&debug, &runtime, &ramp, 0.0f, 0.001f);
    Check(runtime.mode_changed == 0U,
          "stable mode not reported as changed");

    debug.mode = (WheelDebugMode_t)99;
    WheelDebug_Update(&debug, &runtime, &ramp, 0.0f, 0.001f);
    Check(debug.mode == WHEEL_DEBUG_DISABLED,
          "invalid mode falls back to disabled");
    Check(debug.enable == 0U,
          "invalid mode clears enable");
}

static void Test_PositionOptimizationAndRamp(void)
{
    WheelDebug_t debug;
    WheelDebugRuntime_t runtime;
    SetpointRamp_t ramp;

    InitDebug(&debug, &runtime, &ramp);
    debug.mode = WHEEL_DEBUG_GM_POSITION_STEP;
    debug.target_angle_deg = 100.0f;
    debug.direction_erpm = 100;
    debug.use_ramp = 0U;
    WheelDebug_Update(&debug, &runtime, &ramp, 0.0f, 0.001f);
    debug.enable = 1U;
    WheelDebug_Update(&debug, &runtime, &ramp, 0.0f, 0.001f);

    Check(runtime.angle_result.reversed == 1U,
          "positive direction ERPM selects reverse candidate");
    Check(fabsf(runtime.ramp_output_angle_deg - 280.0f) < 0.01f,
          "no-ramp position uses optimized target");

    debug.direction_erpm = 0;
    WheelDebug_Update(&debug, &runtime, &ramp, 0.0f, 0.001f);
    Check(runtime.angle_result.reversed == 1U,
          "zero direction ERPM still selects closer candidate");
    Check(fabsf(runtime.ramp_output_angle_deg - 280.0f) < 0.01f,
          "zero direction ERPM uses optimized target");

    debug.use_ramp = 1U;
    WheelDebug_Update(&debug, &runtime, &ramp, 0.0f, 0.001f);
    Check(debug.enable == 1U,
          "ramp toggle preserves enable");
    Check((fabsf(runtime.ramp_output_angle_deg) < 1.0f) ||
          (fabsf(runtime.ramp_output_angle_deg - 360.0f) < 1.0f),
          "ramp toggle restarts from current angle");
}

static void Test_SineLimits(void)
{
    WheelDebug_t debug;
    WheelDebugRuntime_t runtime;
    SetpointRamp_t ramp;
    uint32_t i;

    InitDebug(&debug, &runtime, &ramp);
    debug.mode = WHEEL_DEBUG_GM_POSITION_SINE;
    debug.use_ramp = 0U;
    debug.wave_amplitude_deg = 500.0f;
    debug.wave_period_ms = 1U;
    WheelDebug_Update(&debug, &runtime, &ramp, 10.0f, 0.001f);
    debug.enable = 1U;
    WheelDebug_Update(&debug, &runtime, &ramp, 10.0f, 0.001f);

    Check(fabsf(runtime.source_angle_deg - 10.0f) < 0.01f,
          "sine starts at current position");

    for (i = 0U; i < 100U; i++)
    {
        WheelDebug_Update(&debug, &runtime, &ramp,
                          10.0f, 0.001f);
        Check(runtime.source_angle_deg >= 0.0f,
              "sine angle normalized low bound");
        Check(runtime.source_angle_deg < 360.0f,
              "sine angle normalized high bound");
    }

    Check(fabsf(debug.wave_amplitude_deg - 500.0f) < 0.001f,
          "watch amplitude value not overwritten");
}

int main(void)
{
    Test_InitAndModeChange();
    Test_PositionOptimizationAndRamp();
    Test_SineLimits();

    if (g_failures != 0U)
    {
        printf("WheelDebug host tests: %u failure(s)\n",
               (unsigned)g_failures);
        return 1;
    }

    printf("WheelDebug host tests: all passed\n");
    return 0;
}
