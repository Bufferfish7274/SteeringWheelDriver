#include "bench_input.h"

#ifdef APP_BENCH

#include <float.h>
#include <math.h>
#include <string.h>

#include "steering_wheel_app.h"

#define BENCH_TWO_PI 6.28318530717958647692f

volatile BenchInput_t g_bench_input;

static uint8_t g_bench_rearm_required;
static uint8_t g_bench_scene_started;
static uint8_t g_bench_previous_enable;
static AppBenchMode_t g_bench_previous_mode;
static float g_bench_origin_angle_deg;
static float g_bench_phase;
static float g_bench_last_target_angle_deg;
static int32_t g_bench_last_target_erpm;
static uint8_t g_bench_pending_target;

static float BenchInput_SanitizeFloat(float value, float fallback)
{
    if ((value != value) ||
        (value > FLT_MAX) ||
        (value < -FLT_MAX))
    {
        return fallback;
    }
    return value;
}

static void BenchInput_ClearSnapshot(AppInputSnapshot_t *input)
{
    memset(input, 0, sizeof(*input));
    input->kind = APP_COMMAND_MOTION;
}

void BenchInput_Reset(const AppConfig_t *config)
{
    (void)config;
    g_bench_input.mode = APP_BENCH_DISABLED;
    g_bench_input.enable = 0U;
    g_bench_input.target_angle_deg = 0.0f;
    g_bench_input.target_erpm = 0;
    g_bench_input.target_rpm = 50.0f;
    g_bench_input.target_voltage = 0;
    g_bench_input.wave_amplitude_deg =
        APP_BENCH_DEFAULT_WAVE_AMPLITUDE_DEG;
    g_bench_input.wave_period_ms =
        APP_BENCH_DEFAULT_WAVE_PERIOD_MS;
    g_bench_input.generated_angle_deg = 0.0f;
    g_bench_input.generated_erpm = 0;

    g_bench_rearm_required = 1U;
    g_bench_scene_started = 0U;
    g_bench_previous_enable = 0U;
    g_bench_previous_mode = APP_BENCH_DISABLED;
    g_bench_origin_angle_deg = 0.0f;
    g_bench_phase = 0.0f;
    g_bench_last_target_angle_deg = 0.0f;
    g_bench_last_target_erpm = 0;
    g_bench_pending_target = 0U;
}

void BenchInput_ForceRearm(void)
{
    g_bench_rearm_required = 1U;
    g_bench_scene_started = 0U;
}

void BenchInput_TargetConsumed(void)
{
    g_bench_pending_target = 0U;
}

static void BenchInput_UpdateWave(float amplitude_deg,
                                  uint32_t period_ms,
                                  uint8_t triangle,
                                  float *target_angle_deg)
{
    float wave;

    if (period_ms < APP_BENCH_MIN_WAVE_PERIOD_MS)
    {
        period_ms = APP_BENCH_MIN_WAVE_PERIOD_MS;
    }

    g_bench_phase += 1.0f / (float)period_ms;
    if (g_bench_phase >= 1.0f)
    {
        g_bench_phase -= 1.0f;
    }

    if (triangle != 0U)
    {
        if (g_bench_phase < 0.25f)
        {
            wave = 4.0f * g_bench_phase;
        }
        else if (g_bench_phase < 0.75f)
        {
            wave = 2.0f - (4.0f * g_bench_phase);
        }
        else
        {
            wave = (4.0f * g_bench_phase) - 4.0f;
        }
    }
    else
    {
        wave = sinf(BENCH_TWO_PI * g_bench_phase);
    }

    *target_angle_deg =
        g_bench_origin_angle_deg + (amplitude_deg * wave);
}

void BenchInput_Build(const AppConfig_t *config,
                      float current_angle_deg,
                      AppInputSnapshot_t *input,
                      uint32_t now_ms)
{
    AppBenchMode_t mode;
    uint8_t enable;
    float amplitude_deg;
    uint32_t period_ms;
    float target_angle_deg;
    int32_t target_erpm;
    float requested_angle_deg;
    float requested_rpm;

    (void)now_ms;

    if ((config == NULL) || (input == NULL))
    {
        return;
    }

    mode = g_bench_input.mode;
    enable = (g_bench_input.enable != 0U) ? 1U : 0U;
    BenchInput_ClearSnapshot(input);

    if ((mode != g_bench_previous_mode) ||
        (mode == APP_BENCH_DISABLED) ||
        !enable)
    {
        BenchInput_ForceRearm();
    }

    if (g_bench_rearm_required &&
        enable &&
        (g_bench_previous_enable == 0U))
    {
        g_bench_rearm_required = 0U;
    }

    g_bench_previous_mode = mode;
    g_bench_previous_enable = enable;

    if ((mode == APP_BENCH_DISABLED) ||
        g_bench_rearm_required)
    {
        g_bench_scene_started = 0U;
        g_bench_input.generated_angle_deg = 0.0f;
        g_bench_input.generated_erpm = 0;
        return;
    }

    input->source_ready = 1U;

    if (!g_bench_scene_started)
    {
        g_bench_origin_angle_deg = current_angle_deg;
        g_bench_phase = 0.0f;
        g_bench_last_target_angle_deg =
            g_bench_input.target_angle_deg;
        g_bench_last_target_erpm = g_bench_input.target_erpm;
        g_bench_scene_started = 1U;
        g_bench_pending_target = 1U;
    }

    amplitude_deg = fabsf(g_bench_input.wave_amplitude_deg);
    amplitude_deg = BenchInput_SanitizeFloat(amplitude_deg, 0.0f);
    if (amplitude_deg > APP_BENCH_MAX_WAVE_AMPLITUDE_DEG)
    {
        amplitude_deg = APP_BENCH_MAX_WAVE_AMPLITUDE_DEG;
    }
    period_ms = g_bench_input.wave_period_ms;
    requested_angle_deg = BenchInput_SanitizeFloat(
        g_bench_input.target_angle_deg,
        current_angle_deg);
    requested_rpm = BenchInput_SanitizeFloat(
        g_bench_input.target_rpm,
        0.0f);

    switch (mode)
    {
        case APP_BENCH_POSITION_MANUAL:
            input->kind = APP_COMMAND_MOTION;
            input->wheel_target_angle_deg = requested_angle_deg;
            input->drive_erpm =
                (g_bench_input.target_erpm > config->bench_vesc_erpm_limit)
                    ? config->bench_vesc_erpm_limit
                    : ((g_bench_input.target_erpm <
                        -config->bench_vesc_erpm_limit)
                           ? -config->bench_vesc_erpm_limit
                           : g_bench_input.target_erpm);
            if ((input->wheel_target_angle_deg !=
                 g_bench_last_target_angle_deg) ||
                (input->drive_erpm != g_bench_last_target_erpm))
            {
                g_bench_pending_target = 1U;
            }
            g_bench_last_target_angle_deg =
                input->wheel_target_angle_deg;
            g_bench_last_target_erpm = input->drive_erpm;
            g_bench_input.generated_angle_deg =
                input->wheel_target_angle_deg;
            g_bench_input.generated_erpm = input->drive_erpm;
            input->required_actuators = APP_ACTUATOR_GM6020;
            if (input->drive_erpm != 0)
            {
                input->required_actuators |= APP_ACTUATOR_VESC;
            }
            break;

        case APP_BENCH_POSITION_SINE:
        case APP_BENCH_POSITION_TRIANGLE:
            BenchInput_UpdateWave(
                amplitude_deg,
                period_ms,
                (mode == APP_BENCH_POSITION_TRIANGLE) ? 1U : 0U,
                &target_angle_deg);
            input->kind = APP_COMMAND_MOTION;
            input->wheel_target_angle_deg = target_angle_deg;
            input->drive_erpm = 0;
            g_bench_pending_target = 1U;
            input->required_actuators = APP_ACTUATOR_GM6020;
            g_bench_input.generated_angle_deg = target_angle_deg;
            g_bench_input.generated_erpm = 0;
            break;

        case APP_BENCH_GM6020_SPEED:
            input->kind = APP_COMMAND_GM6020_SPEED;
            input->gm6020_target_rpm = requested_rpm;
            g_bench_pending_target = 1U;
            input->required_actuators = APP_ACTUATOR_GM6020;
            g_bench_input.generated_angle_deg = 0.0f;
            g_bench_input.generated_erpm = 0;
            break;

        case APP_BENCH_GM6020_VOLTAGE:
            input->kind = APP_COMMAND_GM6020_VOLTAGE;
            input->gm6020_target_voltage =
                g_bench_input.target_voltage;
            g_bench_pending_target = 1U;
            input->required_actuators = APP_ACTUATOR_GM6020;
            g_bench_input.generated_angle_deg = 0.0f;
            g_bench_input.generated_erpm = 0;
            break;

        case APP_BENCH_VESC_ERPM:
            target_erpm = g_bench_input.target_erpm;
            if (target_erpm > config->bench_vesc_erpm_limit)
            {
                target_erpm = config->bench_vesc_erpm_limit;
            }
            else if (target_erpm < -config->bench_vesc_erpm_limit)
            {
                target_erpm = -config->bench_vesc_erpm_limit;
            }
            input->kind = APP_COMMAND_VESC_ERPM;
            input->vesc_target_erpm = target_erpm;
            g_bench_pending_target = 1U;
            input->required_actuators = APP_ACTUATOR_VESC;
            g_bench_input.generated_angle_deg = 0.0f;
            g_bench_input.generated_erpm = target_erpm;
            break;

        default:
            g_bench_pending_target = 0U;
            BenchInput_ClearSnapshot(input);
            break;
    }

    input->has_new_target = g_bench_pending_target;
}

#endif /* APP_BENCH */
