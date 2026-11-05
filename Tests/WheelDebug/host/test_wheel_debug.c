#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "WheelDebug.h"

static uint32_t g_failures;

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

    WheelDebug_Init(&debug, &runtime);
    Check(debug.mode == WHEEL_DEBUG_DISABLED,
          "debug defaults to disabled");
    Check(debug.enable == 0U, "debug defaults to disarmed");
    Check(debug.use_ramp == 1U, "debug defaults to ramp enabled");

    debug.mode = WHEEL_DEBUG_GM_POSITION_STEP;
    debug.enable = 1U;
    WheelDebug_Update(&debug, &runtime, 0.0f, 0.001f);
    Check(runtime.mode_changed == 1U,
          "mode change reported");
    Check(debug.enable == 0U,
          "mode change clears debug enable");

    debug.enable = 1U;
    WheelDebug_Update(&debug, &runtime, 0.0f, 0.001f);
    Check(runtime.mode_changed == 0U,
          "stable mode not reported as changed");

    debug.mode = (WheelDebugMode_t)99;
    WheelDebug_Update(&debug, &runtime, 0.0f, 0.001f);
    Check(debug.mode == WHEEL_DEBUG_DISABLED,
          "invalid mode falls back to disabled");
    Check(debug.enable == 0U,
          "invalid mode clears enable");
}

static void Test_PositionOptimizationAndRamp(void)
{
    WheelDebug_t debug;
    WheelDebugRuntime_t runtime;

    WheelDebug_Init(&debug, &runtime);
    debug.mode = WHEEL_DEBUG_GM_POSITION_STEP;
    debug.target_angle_deg = 100.0f;
    debug.direction_erpm = 100;
    debug.use_ramp = 0U;
    WheelDebug_Update(&debug, &runtime, 0.0f, 0.001f);
    debug.enable = 1U;
    WheelDebug_Update(&debug, &runtime, 0.0f, 0.001f);

    Check(runtime.angle_result.reversed == 1U,
          "positive direction ERPM selects reverse candidate");
    Check(fabsf(runtime.ramp_output_angle_deg - 280.0f) < 0.01f,
          "no-ramp position uses optimized target");

    debug.direction_erpm = 0;
    WheelDebug_Update(&debug, &runtime, 0.0f, 0.001f);
    Check(runtime.angle_result.reversed == 0U,
          "zero direction ERPM keeps original target");
    Check(fabsf(runtime.ramp_output_angle_deg - 100.0f) < 0.01f,
          "zero ERPM position target is unchanged");

    debug.use_ramp = 1U;
    WheelDebug_Update(&debug, &runtime, 0.0f, 0.001f);
    Check(debug.enable == 1U,
          "ramp toggle preserves enable");
    Check(fabsf(runtime.ramp_output_angle_deg) < 1.0f,
          "ramp toggle restarts from current angle");
}

static void Test_SineLimits(void)
{
    WheelDebug_t debug;
    WheelDebugRuntime_t runtime;
    uint32_t i;

    WheelDebug_Init(&debug, &runtime);
    debug.mode = WHEEL_DEBUG_GM_POSITION_SINE;
    debug.use_ramp = 0U;
    debug.wave_amplitude_deg = 500.0f;
    debug.wave_period_ms = 1U;
    WheelDebug_Update(&debug, &runtime, 10.0f, 0.001f);
    debug.enable = 1U;
    WheelDebug_Update(&debug, &runtime, 10.0f, 0.001f);

    Check(fabsf(runtime.source_angle_deg - 10.0f) < 0.01f,
          "sine starts at current position");

    for (i = 0U; i < 100U; i++)
    {
        WheelDebug_Update(&debug, &runtime, 10.0f, 0.001f);
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
