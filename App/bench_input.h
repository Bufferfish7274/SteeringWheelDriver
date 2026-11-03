#ifndef BENCH_INPUT_H
#define BENCH_INPUT_H

#include <stdint.h>

#include "steering_wheel_app.h"

#ifdef APP_BENCH

typedef enum
{
    APP_BENCH_DISABLED = 0,
    APP_BENCH_POSITION_MANUAL,
    APP_BENCH_POSITION_SINE,
    APP_BENCH_POSITION_TRIANGLE,
    APP_BENCH_GM6020_SPEED,
    APP_BENCH_GM6020_VOLTAGE,
    APP_BENCH_VESC_ERPM
} AppBenchMode_t;

typedef struct
{
    volatile AppBenchMode_t mode;
    volatile uint8_t enable;
    volatile float target_angle_deg;
    volatile int32_t target_erpm;
    volatile float target_rpm;
    volatile int16_t target_voltage;
    volatile float wave_amplitude_deg;
    volatile uint32_t wave_period_ms;
    volatile float generated_angle_deg;
    volatile int32_t generated_erpm;
} BenchInput_t;

extern volatile BenchInput_t g_bench_input;

void BenchInput_Reset(const AppConfig_t *config);

void BenchInput_ForceRearm(void);

void BenchInput_TargetConsumed(void);

void BenchInput_Build(const AppConfig_t *config,
                      float current_angle_deg,
                      AppInputSnapshot_t *input,
                      uint32_t now_ms);

#endif /* APP_BENCH */

#endif /* BENCH_INPUT_H */
