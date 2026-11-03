#include "steering_wheel_app.h"

#include <math.h>
#include <string.h>

static float App_Wrap360(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static uint32_t App_ElapsedMs(uint32_t now_ms, uint32_t timestamp_ms)
{
    return (uint32_t)(now_ms - timestamp_ms);
}

#ifdef APP_BENCH
static int16_t App_ClampInt16(int32_t value, int16_t limit)
{
    int32_t magnitude = limit;

    if (magnitude < 0)
    {
        magnitude = -magnitude;
    }

    if (value > magnitude)
    {
        return (int16_t)magnitude;
    }
    if (value < -magnitude)
    {
        return (int16_t)(-magnitude);
    }
    return (int16_t)value;
}

static int32_t App_ClampInt32(int32_t value, int32_t limit)
{
    if (limit < 0)
    {
        limit = -limit;
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
#endif

static void App_CopyPidTuning(Pids *pid, const AppPidConfig_t *config)
{
    pid->kp = config->kp;
    pid->ki = config->ki;
    pid->kd = config->kd;
    pid->dt = APP_CONTROL_DT_S;
    pid->integral_max = config->integral_max;
    pid->integral_separation = config->integral_separation;
    pid->separation_threshold = config->separation_threshold;
}

static void App_ApplyLiveConfig(SteeringWheelApp_t *app)
{
    App_CopyPidTuning(&app->gm6020.position_pid,
                      &app->config.position_pid);
    App_CopyPidTuning(&app->gm6020.speed_pid,
                      &app->config.speed_pid);

    app->gm6020.position_pid.output_max =
        fabsf(app->config.max_speed_rpm);
    app->gm6020.speed_pid.output_max =
        fabsf((float)app->config.voltage_limit_raw);

    app->gm6020.max_speed_rpm = app->config.max_speed_rpm;
    app->gm6020.voltage_limit_raw = app->config.voltage_limit_raw;
    app->gm6020.online_timeout_ms = app->config.gm6020_timeout_ms;
    app->vesc.online_timeout_ms = app->config.vesc_timeout_ms;

    app->ramp.max_speed_deg_s =
        app->config.ramp_max_speed_deg_s;
    app->ramp.max_accel_deg_s2 =
        app->config.ramp_max_accel_deg_s2;
}

static void App_ResetControllers(SteeringWheelApp_t *app)
{
    PidReset(&app->gm6020.position_pid);
    PidReset(&app->gm6020.speed_pid);
    SetpointRamp_Reset(&app->ramp, app->gm6020.angle_deg);
}

static void App_DisableOutputs(SteeringWheelApp_t *app)
{
    if (app->gm6020.is_enabled)
    {
        GM6020_Disable(&app->gm6020);
    }

    if (app->vesc.is_enabled)
    {
        VESC_Disable(&app->vesc);
    }

    app->control_enabled = 0U;
    app->need_fresh_control = 1U;
}

static void App_InitObjects(SteeringWheelApp_t *app)
{
    GM6020_Init(&app->gm6020,
                app->can1,
                app->config.gm6020_id,
                app->config.gm6020_zero_raw,
                app->config.gm6020_angle_sign,
                app->config.gm6020_voltage_sign,
                app->config.gm6020_gear_ratio);
    VESC_Init(&app->vesc,
              app->can1,
              app->config.vesc_controller_id);
    SetpointRamp_Init(&app->ramp,
                      0.0f,
                      app->config.ramp_max_speed_deg_s,
                      app->config.ramp_max_accel_deg_s2);
    App_ApplyLiveConfig(app);
}

static uint8_t App_ApplyTuningRequest(SteeringWheelApp_t *app)
{
    AppConfig_t requested;
    uint32_t sequence = g_tuning_apply_seq;
    uint8_t structural;
    uint8_t mapping_changed;

    if (sequence == app->applied_config_seq)
    {
        return 1U;
    }

    requested = g_tuning_config;
    app->applied_config_seq = sequence;

    if (!AppConfig_Validate(&requested))
    {
        app->config_apply_status = APP_CONFIG_APPLY_INVALID;
        g_tuning_apply_status = APP_CONFIG_APPLY_INVALID;
        g_tuning_reset_controllers = 0U;
        return 0U;
    }

    structural = AppConfig_RequiresStructuralReinit(
        &app->config, &requested);
    if (structural && app->control_enabled)
    {
        app->config_apply_status =
            APP_CONFIG_APPLY_REJECTED_ACTIVE;
        g_tuning_apply_status =
            APP_CONFIG_APPLY_REJECTED_ACTIVE;
        g_tuning_reset_controllers = 0U;
        return 0U;
    }

    mapping_changed = AppConfig_MappingChanged(
        &app->config, &requested);

    if (structural || mapping_changed)
    {
        App_DisableOutputs(app);
    }
    app->config = requested;

    if (structural)
    {
        App_InitObjects(app);
        if ((app->structural_apply != NULL) &&
            (app->structural_apply(app->structural_context,
                                   &app->config) != HAL_OK))
        {
            app->config_apply_status = APP_CONFIG_APPLY_BUS_ERROR;
            g_tuning_apply_status = APP_CONFIG_APPLY_BUS_ERROR;
            g_tuning_reset_controllers = 0U;
            app->need_fresh_control = 1U;
            return 2U;
        }
    }
    else
    {
        App_ApplyLiveConfig(app);
    }

    if ((g_tuning_reset_controllers != 0U) ||
        structural ||
        mapping_changed)
    {
        App_ResetControllers(app);
    }

    g_tuning_reset_controllers = 0U;
    app->config_apply_status = APP_CONFIG_APPLY_OK;
    g_tuning_apply_status = APP_CONFIG_APPLY_OK;
    g_tuning_applied_seq = sequence;
    if (structural || mapping_changed)
    {
        app->need_fresh_control = 1U;
        return 2U;
    }
    return 1U;
}

static void App_UpdateOnlineState(SteeringWheelApp_t *app,
                                  uint32_t now_ms)
{
    app->gm6020_online =
        (app->gm6020.is_online &&
         (App_ElapsedMs(now_ms, app->gm6020.last_rx_ms) <=
          app->gm6020.online_timeout_ms)) ? 1U : 0U;
    app->vesc_online =
        (app->vesc.is_online &&
         (App_ElapsedMs(now_ms, app->vesc.last_rx_ms) <=
          app->vesc.online_timeout_ms)) ? 1U : 0U;
}

static uint8_t App_RequiredActuatorsOnline(
    const SteeringWheelApp_t *app,
    uint8_t required)
{
    if ((required &
         (APP_ACTUATOR_GM6020 | APP_ACTUATOR_VESC)) == 0U)
    {
        return 0U;
    }
    if (((required & APP_ACTUATOR_GM6020) != 0U) &&
        !app->gm6020_online)
    {
        return 0U;
    }
    if (((required & APP_ACTUATOR_VESC) != 0U) &&
        !app->vesc_online)
    {
        return 0U;
    }
    return 1U;
}

static void App_EnableRequiredActuators(SteeringWheelApp_t *app,
                                        uint8_t required)
{
    if ((required & APP_ACTUATOR_GM6020) != 0U)
    {
        if (!app->gm6020.is_enabled)
        {
            GM6020_Enable(&app->gm6020);
        }
    }
    else if (app->gm6020.is_enabled)
    {
        GM6020_Disable(&app->gm6020);
    }

    if ((required & APP_ACTUATOR_VESC) != 0U)
    {
        if (!app->vesc.is_enabled)
        {
            VESC_Enable(&app->vesc);
        }
    }
    else if (app->vesc.is_enabled)
    {
        VESC_Disable(&app->vesc);
    }
}

static void App_AcceptTarget(SteeringWheelApp_t *app,
                             const AppInputSnapshot_t *input)
{
    AngleOptResult_t result;

    if (input->kind == APP_COMMAND_MOTION)
    {
        AngleOpt_Select(app->gm6020.angle_deg,
                        input->wheel_target_angle_deg,
                        input->drive_erpm,
                        &result);

        if (!app->control_enabled)
        {
            SetpointRamp_Reset(&app->ramp, app->gm6020.angle_deg);
        }

        app->mapped_target_deg =
            App_Wrap360(input->wheel_target_angle_deg);
        app->final_target_angle_deg = result.target_angle_deg;
        app->final_target_erpm = result.target_erpm;
        app->angle_reversed = result.reversed;

        SetpointRamp_SetDesired(&app->ramp,
                                result.target_angle_deg);
        GM6020_SetTargetAngle(&app->gm6020,
                              SetpointRamp_GetOutput(&app->ramp));
        VESC_SetTargetErpm(&app->vesc, result.target_erpm);
    }
#ifdef APP_BENCH
    else if (input->kind == APP_COMMAND_GM6020_SPEED)
    {
        app->gm6020.target_speed_rpm = input->gm6020_target_rpm;
    }
    else if (input->kind == APP_COMMAND_GM6020_VOLTAGE)
    {
        app->gm6020.target_voltage =
            App_ClampInt16(input->gm6020_target_voltage,
                           app->config.bench_gm_voltage_limit_raw);
    }
    else
    {
        VESC_SetTargetErpm(
            &app->vesc,
            App_ClampInt32(input->vesc_target_erpm,
                           app->config.bench_vesc_erpm_limit));
    }
#endif

    App_EnableRequiredActuators(app, input->required_actuators);
    app->control_enabled = 1U;
    app->need_fresh_control = 0U;
}

static void App_RunActiveControl(SteeringWheelApp_t *app,
                                 const AppInputSnapshot_t *input)
{
    if (input->kind == APP_COMMAND_MOTION)
    {
        SetpointRamp_Update(&app->ramp, APP_CONTROL_DT_S);
        GM6020_SetTargetAngle(&app->gm6020,
                              SetpointRamp_GetOutput(&app->ramp));
        GM6020_ControlStep(&app->gm6020, app->last_now_ms);
        if ((input->required_actuators &
             APP_ACTUATOR_VESC) != 0U)
        {
            VESC_ControlStep(&app->vesc, app->last_now_ms);
        }
    }
#ifdef APP_BENCH
    else if (input->kind == APP_COMMAND_GM6020_SPEED)
    {
        GM6020_ControlSpeedStep(
            &app->gm6020,
            input->gm6020_target_rpm,
            app->last_now_ms);
        VESC_ControlStep(&app->vesc, app->last_now_ms);
    }
    else if (input->kind == APP_COMMAND_GM6020_VOLTAGE)
    {
        GM6020_ControlVoltageStep(
            &app->gm6020,
            App_ClampInt16(input->gm6020_target_voltage,
                           app->config.bench_gm_voltage_limit_raw),
            app->last_now_ms);
        VESC_ControlStep(&app->vesc, app->last_now_ms);
    }
    else
    {
        GM6020_ControlStep(&app->gm6020, app->last_now_ms);
        VESC_ControlStep(&app->vesc, app->last_now_ms);
    }
#endif
}

static void App_RunDisabledControl(SteeringWheelApp_t *app)
{
    GM6020_ControlStep(&app->gm6020, app->last_now_ms);
    VESC_ControlStep(&app->vesc, app->last_now_ms);
}

void SteeringWheelApp_Init(SteeringWheelApp_t *app,
                           CAN_HandleTypeDef *can1,
                           const AppConfig_t *config,
                           AppStructuralApplyCallback_t structural_apply,
                           void *structural_context)
{
    if ((app == NULL) || (can1 == NULL) || (config == NULL))
    {
        Error_Handler();
        return;
    }

    memset(app, 0, sizeof(*app));
    app->can1 = can1;
    app->config = *config;
    app->structural_apply = structural_apply;
    app->structural_context = structural_context;
    app->need_fresh_control = 1U;
    app->config_apply_status = APP_CONFIG_APPLY_NONE;
    app->last_now_ms = 0U;
    App_InitObjects(app);
}

void SteeringWheelApp_OnCan1Rx(SteeringWheelApp_t *app,
                               const CAN_RxHeaderTypeDef *header,
                               const uint8_t data[8],
                               uint32_t now_ms)
{
    if (app == NULL)
    {
        return;
    }

    GM6020_OnRx(&app->gm6020, header, data, now_ms);
    VESC_OnRx(&app->vesc, header, data, now_ms);
}

void SteeringWheelApp_ControlStep(SteeringWheelApp_t *app,
                                  AppInputSnapshot_t *input,
                                  uint32_t now_ms)
{
    uint8_t config_result;
    uint8_t required_online;

    if ((app == NULL) || (input == NULL))
    {
        return;
    }

    app->last_now_ms = now_ms;
    app->required_actuators = input->required_actuators;
    config_result = App_ApplyTuningRequest(app);
    App_UpdateOnlineState(app, now_ms);

    if (config_result == 2U)
    {
        input->has_new_target = 0U;
        App_RunDisabledControl(app);
        return;
    }

    if (now_ms < APP_SELF_TEST_MS)
    {
        App_DisableOutputs(app);
        App_RunDisabledControl(app);
        return;
    }

    required_online = App_RequiredActuatorsOnline(
        app, input->required_actuators);

    if (!input->source_ready || !required_online)
    {
        input->has_new_target = 0U;
        App_DisableOutputs(app);
        App_RunDisabledControl(app);
        return;
    }

    if (input->has_new_target)
    {
        App_AcceptTarget(app, input);
        input->has_new_target = 0U;
    }

    if (app->control_enabled && !app->need_fresh_control)
    {
        App_RunActiveControl(app, input);
    }
    else
    {
        App_RunDisabledControl(app);
    }
}

void SteeringWheelApp_RequestDisable(SteeringWheelApp_t *app)
{
    if (app != NULL)
    {
        App_DisableOutputs(app);
    }
}

void SteeringWheelApp_GetSnapshot(const SteeringWheelApp_t *app,
                                  AppSnapshot_t *snapshot)
{
    if ((app == NULL) || (snapshot == NULL))
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->boot_complete =
        (app->last_now_ms >= APP_SELF_TEST_MS) ? 1U : 0U;
    snapshot->control_enabled = app->control_enabled;
    snapshot->need_fresh_control = app->need_fresh_control;
    snapshot->gm6020_online = app->gm6020_online;
    snapshot->vesc_online = app->vesc_online;
    snapshot->angle_reversed = app->angle_reversed;
    snapshot->required_actuators = app->required_actuators;
    snapshot->mapped_target_deg = app->mapped_target_deg;
    snapshot->final_target_angle_deg = app->final_target_angle_deg;
    snapshot->final_target_erpm = app->final_target_erpm;
    snapshot->applied_config_seq = g_tuning_applied_seq;
    snapshot->config_apply_status = app->config_apply_status;

    if (((app->required_actuators & APP_ACTUATOR_GM6020) != 0U) &&
        !app->gm6020_online)
    {
        snapshot->fault_flags |= APP_FAULT_GM6020_OFFLINE;
    }
    if (((app->required_actuators & APP_ACTUATOR_VESC) != 0U) &&
        !app->vesc_online)
    {
        snapshot->fault_flags |= APP_FAULT_VESC_OFFLINE;
    }
    if (app->need_fresh_control)
    {
        snapshot->fault_flags |= APP_FAULT_NEED_FRESH;
    }
    if (((app->required_actuators & APP_ACTUATOR_GM6020) != 0U) &&
        (app->gm6020.last_tx_status != HAL_OK))
    {
        snapshot->fault_flags |= APP_FAULT_GM6020_TX;
    }
    if (((app->required_actuators & APP_ACTUATOR_VESC) != 0U) &&
        (app->vesc.last_tx_status != HAL_OK))
    {
        snapshot->fault_flags |= APP_FAULT_VESC_TX;
    }
    if ((app->config_apply_status == APP_CONFIG_APPLY_INVALID) ||
        (app->config_apply_status == APP_CONFIG_APPLY_BUS_ERROR))
    {
        snapshot->fault_flags |= APP_FAULT_CONFIG;
    }
}

const AppConfig_t *SteeringWheelApp_GetEffectiveConfig(
    const SteeringWheelApp_t *app)
{
    return (app == NULL) ? NULL : &app->config;
}

float SteeringWheelApp_MapMasterAngle(const AppConfig_t *config,
                                      float master_angle_deg)
{
    if (config == NULL)
    {
        return 0.0f;
    }

    return App_Wrap360(
        (config->master_angle_sign * master_angle_deg) +
        config->master_angle_offset_deg);
}

uint8_t SteeringWheelApp_IsActive(const SteeringWheelApp_t *app)
{
    return ((app != NULL) && app->control_enabled) ? 1U : 0U;
}
