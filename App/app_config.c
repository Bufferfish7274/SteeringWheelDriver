#include "app_config.h"

#include <float.h>

#include "GM6020.h"
#include "MasterCan.h"
#include "VESC.h"
#include "setpoint_ramp.h"

volatile AppConfig_t g_tuning_config;
volatile uint32_t g_tuning_apply_seq;
volatile uint8_t g_tuning_reset_controllers;
volatile AppConfigApplyStatus_t g_tuning_apply_status;
volatile uint32_t g_tuning_applied_seq;

static uint8_t AppConfig_IsFinite(float value)
{
    return ((value == value) &&
            (value <= FLT_MAX) &&
            (value >= -FLT_MAX)) ? 1U : 0U;
}

static uint8_t AppConfig_PidValid(const AppPidConfig_t *pid)
{
    if ((pid == NULL) ||
        !AppConfig_IsFinite(pid->kp) ||
        !AppConfig_IsFinite(pid->ki) ||
        !AppConfig_IsFinite(pid->kd) ||
        !AppConfig_IsFinite(pid->integral_max) ||
        !AppConfig_IsFinite(pid->separation_threshold))
    {
        return 0U;
    }

    if ((pid->kp < 0.0f) ||
        (pid->ki < 0.0f) ||
        (pid->kd < 0.0f) ||
        (pid->integral_max < 0.0f) ||
        (pid->separation_threshold < 0.0f))
    {
        return 0U;
    }

    return (pid->integral_separation <= 1U) ? 1U : 0U;
}

void AppConfig_LoadDefaults(AppConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->master_wheel_id = 1U;
    config->gm6020_id = 1U;
    config->vesc_controller_id = 67U;

    config->gm6020_zero_raw = 1386U;
    config->gm6020_angle_sign = 1;
    config->gm6020_voltage_sign = 1;
    config->gm6020_gear_ratio = 1.0f;

    config->master_angle_sign = 1.0f;
    config->master_angle_offset_deg = 0.0f;

    config->position_pid.kp = GM6020_DEFAULT_POS_KP;
    config->position_pid.ki = GM6020_DEFAULT_POS_KI;
    config->position_pid.kd = GM6020_DEFAULT_POS_KD;
    config->position_pid.integral_max =
        GM6020_DEFAULT_POS_INTEGRAL_MAX;
    config->position_pid.integral_separation = 1U;
    config->position_pid.separation_threshold =
        GM6020_DEFAULT_POS_SEP_DEG;

    config->speed_pid.kp = GM6020_DEFAULT_SPD_KP;
    config->speed_pid.ki = GM6020_DEFAULT_SPD_KI;
    config->speed_pid.kd = GM6020_DEFAULT_SPD_KD;
    config->speed_pid.integral_max =
        GM6020_DEFAULT_SPD_INTEGRAL_MAX;
    config->speed_pid.integral_separation = 0U;
    config->speed_pid.separation_threshold =
        GM6020_DEFAULT_SPD_SEP_RPM;

    config->max_speed_rpm = GM6020_DEFAULT_MAX_SPEED_RPM;
    config->voltage_limit_raw = GM6020_DEFAULT_VOLTAGE_LIMIT;
    config->ramp_max_speed_deg_s =
        SETPOINT_RAMP_DEFAULT_MAX_SPEED_DEG_S;
    config->ramp_max_accel_deg_s2 =
        SETPOINT_RAMP_DEFAULT_MAX_ACCEL_DEG_S2;

    config->master_link_timeout_ms = MASTER_LINK_TIMEOUT_MS;
    config->gm6020_timeout_ms = GM6020_DEFAULT_TIMEOUT_MS;
    config->vesc_timeout_ms = VESC_DEFAULT_TIMEOUT_MS;

    config->bench_gm_voltage_limit_raw =
        APP_BENCH_DEFAULT_GM_VOLTAGE;
    config->bench_vesc_erpm_limit =
        APP_BENCH_DEFAULT_VESC_ERPM;
}

void AppConfig_InitTuning(const AppConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    g_tuning_config = *config;
    g_tuning_apply_seq = 0U;
    g_tuning_applied_seq = 0U;
    g_tuning_reset_controllers = 0U;
    g_tuning_apply_status = APP_CONFIG_APPLY_NONE;
}

uint8_t AppConfig_Validate(const AppConfig_t *config)
{
    if (config == NULL)
    {
        return 0U;
    }

    if ((config->master_wheel_id < 1U) ||
        (config->master_wheel_id > 4U) ||
        (config->gm6020_id < 1U) ||
        (config->gm6020_id > 7U) ||
        (config->gm6020_zero_raw > 8191U))
    {
        return 0U;
    }

    if ((config->gm6020_angle_sign != -1) &&
        (config->gm6020_angle_sign != 1))
    {
        return 0U;
    }

    if ((config->gm6020_voltage_sign != -1) &&
        (config->gm6020_voltage_sign != 1))
    {
        return 0U;
    }

    if (!AppConfig_IsFinite(config->gm6020_gear_ratio) ||
        (config->gm6020_gear_ratio <= 0.0f) ||
        !AppConfig_IsFinite(config->master_angle_sign) ||
        !AppConfig_IsFinite(config->master_angle_offset_deg) ||
        ((config->master_angle_sign != -1.0f) &&
         (config->master_angle_sign != 1.0f)))
    {
        return 0U;
    }

    if (!AppConfig_PidValid(&config->position_pid) ||
        !AppConfig_PidValid(&config->speed_pid))
    {
        return 0U;
    }

    if (!AppConfig_IsFinite(config->max_speed_rpm) ||
        (config->max_speed_rpm <= 0.0f) ||
        (config->voltage_limit_raw < 0) ||
        (config->voltage_limit_raw > GM6020_MAX_VOLTAGE_RAW) ||
        !AppConfig_IsFinite(config->ramp_max_speed_deg_s) ||
        (config->ramp_max_speed_deg_s <= 0.0f) ||
        !AppConfig_IsFinite(config->ramp_max_accel_deg_s2) ||
        (config->ramp_max_accel_deg_s2 <= 0.0f))
    {
        return 0U;
    }

    if ((config->master_link_timeout_ms < 1U) ||
        (config->gm6020_timeout_ms < 1U) ||
        (config->vesc_timeout_ms < 1U) ||
        (config->bench_gm_voltage_limit_raw < 0) ||
        (config->bench_gm_voltage_limit_raw >
         GM6020_MAX_VOLTAGE_RAW) ||
        (config->bench_vesc_erpm_limit <= 0))
    {
        return 0U;
    }

    return 1U;
}

uint8_t AppConfig_RequiresStructuralReinit(
    const AppConfig_t *old_config,
    const AppConfig_t *new_config)
{
    if ((old_config == NULL) || (new_config == NULL))
    {
        return 1U;
    }

    return ((old_config->master_wheel_id !=
             new_config->master_wheel_id) ||
            (old_config->gm6020_id != new_config->gm6020_id) ||
            (old_config->vesc_controller_id !=
             new_config->vesc_controller_id) ||
            (old_config->gm6020_zero_raw !=
             new_config->gm6020_zero_raw) ||
            (old_config->gm6020_angle_sign !=
             new_config->gm6020_angle_sign) ||
            (old_config->gm6020_voltage_sign !=
             new_config->gm6020_voltage_sign) ||
            (old_config->gm6020_gear_ratio !=
             new_config->gm6020_gear_ratio)) ? 1U : 0U;
}

uint8_t AppConfig_MappingChanged(const AppConfig_t *old_config,
                                 const AppConfig_t *new_config)
{
    if ((old_config == NULL) || (new_config == NULL))
    {
        return 1U;
    }

    return ((old_config->master_angle_sign !=
             new_config->master_angle_sign) ||
            (old_config->master_angle_offset_deg !=
             new_config->master_angle_offset_deg)) ? 1U : 0U;
}
