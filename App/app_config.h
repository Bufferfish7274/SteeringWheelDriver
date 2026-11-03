#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/**
 * @file app_config.h
 * @brief 舵轮应用运行参数和 Keil Watch 调参入口。
 *
 * 配置只保存在 RAM 中。修改 g_tuning_config 后递增
 * g_tuning_apply_seq，控制节拍会校验并应用一次。
 */

#define APP_CONTROL_DT_S                0.001f
#define APP_SELF_TEST_MS                500U
#define APP_TELEMETRY_MS                20U
#define APP_LED_UPDATE_MS               10U

#define APP_BENCH_DEFAULT_VESC_ERPM     5000
#define APP_BENCH_DEFAULT_GM_VOLTAGE    5000
#define APP_BENCH_DEFAULT_WAVE_AMPLITUDE_DEG 30.0f
#define APP_BENCH_DEFAULT_WAVE_PERIOD_MS 4000U
#define APP_BENCH_MIN_WAVE_PERIOD_MS    200U
#define APP_BENCH_MAX_WAVE_AMPLITUDE_DEG 180.0f

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_max;
    uint8_t integral_separation;
    float separation_threshold;
} AppPidConfig_t;

typedef struct
{
    uint8_t master_wheel_id;
    uint8_t gm6020_id;
    uint8_t vesc_controller_id;

    uint16_t gm6020_zero_raw;
    int8_t gm6020_angle_sign;
    int8_t gm6020_voltage_sign;
    float gm6020_gear_ratio;

    float master_angle_sign;
    float master_angle_offset_deg;

    AppPidConfig_t position_pid;
    AppPidConfig_t speed_pid;
    float max_speed_rpm;
    int16_t voltage_limit_raw;
    float ramp_max_speed_deg_s;
    float ramp_max_accel_deg_s2;

    uint32_t master_link_timeout_ms;
    uint32_t gm6020_timeout_ms;
    uint32_t vesc_timeout_ms;

    int16_t bench_gm_voltage_limit_raw;
    int32_t bench_vesc_erpm_limit;
} AppConfig_t;

typedef enum
{
    APP_CONFIG_APPLY_NONE = 0,
    APP_CONFIG_APPLY_OK,
    APP_CONFIG_APPLY_INVALID,
    APP_CONFIG_APPLY_REJECTED_ACTIVE,
    APP_CONFIG_APPLY_BUS_ERROR
} AppConfigApplyStatus_t;

extern volatile AppConfig_t g_tuning_config;
extern volatile uint32_t g_tuning_apply_seq;
extern volatile uint8_t g_tuning_reset_controllers;
extern volatile AppConfigApplyStatus_t g_tuning_apply_status;
extern volatile uint32_t g_tuning_applied_seq;

/** @brief 装载当前生产值作为配置默认值。 */
void AppConfig_LoadDefaults(AppConfig_t *config);

/** @brief 把默认值装入 Watch 可见配置并清空应用状态。 */
void AppConfig_InitTuning(const AppConfig_t *config);

/** @brief 校验配置的协议、硬件和数值边界。 */
uint8_t AppConfig_Validate(const AppConfig_t *config);

/** @brief 判断两份配置是否需要重新初始化设备和 CAN 过滤器。 */
uint8_t AppConfig_RequiresStructuralReinit(const AppConfig_t *old_config,
                                           const AppConfig_t *new_config);

/** @brief 判断主控坐标映射是否发生变化。 */
uint8_t AppConfig_MappingChanged(const AppConfig_t *old_config,
                                 const AppConfig_t *new_config);

#endif /* APP_CONFIG_H */
