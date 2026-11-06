/**
 * @file App.c
 * @brief 舵轮应用生命周期、实时控制和安全状态管理。
 *
 * 数据流：
 *   正式版：
 *     CAN2 主控命令 -> MasterCan 解码 -> App 安全判断
 *       -> AngleOpt -> SetpointRamp -> GM6020 位置环 + VESC ERPM
 *
 *   调试版：
 *     g_wheel_debug -> WheelDebug 目标生成
 *       -> GM6020 位置/速度/电压 或 VESC ERPM
 *
 * 运行上下文：
 *   - CAN RX 中断：只解码反馈和登记新控制帧；
 *   - TIM6 1 kHz：在线判断、安全状态、目标应用和电机控制；
 *   - 主循环：10 ms LED 和 20 ms 非阻塞串口遥测。
 */

#include "App.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "AppConfig.h"
#include "bsp_can.h"

#if (WHEEL_BUILD_DEBUG != 0) && (WHEEL_BUILD_DEBUG != 1)
#error "WHEEL_BUILD_DEBUG must be 0 or 1"
#endif

static const SetpointRamp_Config_t g_ramp_config =
{
    .max_speed_deg_s = APP_RAMP_MAX_SPEED_DEG_S,
    .max_accel_deg_s2 = APP_RAMP_MAX_ACCEL_DEG_S2
};

static const GM6020_Config_t g_gm6020_config =
{
    .id = APP_GM6020_ID,
    .zero_raw = APP_GM6020_ZERO_RAW,
    .angle_sign = APP_GM6020_ANGLE_SIGN,
    .voltage_sign = APP_GM6020_VOLTAGE_SIGN,
    .gear_ratio = APP_GM6020_GEAR_RATIO,
    .position =
    {
        .kp = APP_GM6020_POS_KP,
        .ki = APP_GM6020_POS_KI,
        .kd = APP_GM6020_POS_KD,
        .integral_max = APP_GM6020_POS_INTEGRAL_MAX,
        .integral_separation =
            APP_GM6020_POS_INTEGRAL_SEPARATE,
        .separation_threshold =
            APP_GM6020_POS_SEPARATION_DEG
    },
    .speed =
    {
        .kp = APP_GM6020_SPEED_KP,
        .ki = APP_GM6020_SPEED_KI,
        .kd = APP_GM6020_SPEED_KD,
        .integral_max = APP_GM6020_SPEED_INTEGRAL_MAX,
        .integral_separation =
            APP_GM6020_SPEED_INTEGRAL_SEPARATE,
        .separation_threshold =
            APP_GM6020_SPEED_SEPARATION_RPM
    },
    .control_dt_s = APP_CONTROL_DT_S,
    .max_speed_rpm = APP_GM6020_MAX_SPEED_RPM,
    .voltage_limit_raw = APP_GM6020_VOLTAGE_LIMIT_RAW,
    .online_timeout_ms = APP_GM6020_ONLINE_TIMEOUT_MS
};

static const VESC_Config_t g_vesc_config =
{
    .controller_id = APP_VESC_CONTROLLER_ID,
    .online_timeout_ms = APP_VESC_ONLINE_TIMEOUT_MS
};

#if !WHEEL_BUILD_DEBUG
static const MasterCan_Config_t g_master_config =
{
    .wheel_id = APP_MASTER_WHEEL_ID,
    .link_timeout_ms = APP_MASTER_LINK_TIMEOUT_MS
};
#else
static const WheelDebug_Config_t g_wheel_debug_config =
{
    .target_angle_deg = APP_DEBUG_DEFAULT_ANGLE_DEG,
    .direction_erpm = APP_DEBUG_DEFAULT_DIRECTION_ERPM,
    .target_speed_rpm = APP_DEBUG_DEFAULT_SPEED_RPM,
    .target_voltage_raw = APP_DEBUG_DEFAULT_VOLTAGE_RAW,
    .target_erpm = APP_DEBUG_DEFAULT_TARGET_ERPM,
    .erpm_limit = APP_DEBUG_DEFAULT_ERPM_LIMIT,
    .use_ramp = APP_DEBUG_DEFAULT_USE_RAMP,
    .wave_amplitude_deg = APP_DEBUG_DEFAULT_AMPLITUDE_DEG,
    .wave_period_ms = APP_DEBUG_DEFAULT_PERIOD_MS,
    .min_period_ms = APP_DEBUG_MIN_PERIOD_MS,
    .max_amplitude_deg = APP_DEBUG_MAX_AMPLITUDE_DEG
};
#endif

/**
 * @brief 计算无符号毫秒时间差。
 * @note 使用 uint32_t 减法，时间戳自然回绕后差值仍然正确。
 */
static uint32_t App_ElapsedMs(uint32_t now_ms,
                              uint32_t timestamp_ms)
{
    return (uint32_t)(now_ms - timestamp_ms);
}

/** @brief 校验一组 PID 配置是否完整且有限。 */
static uint8_t App_IsValidPidConfig(
    const GM6020_PidConfig_t *config)
{
    return ((config != NULL) &&
            isfinite(config->kp) &&
            isfinite(config->ki) &&
            isfinite(config->kd) &&
            (config->kp >= 0.0f) &&
            (config->ki >= 0.0f) &&
            (config->kd >= 0.0f) &&
            isfinite(config->integral_max) &&
            (config->integral_max >= 0.0f) &&
            (config->integral_separation <= 1U) &&
            isfinite(config->separation_threshold) &&
            (config->separation_threshold >= 0.0f)) ? 1U : 0U;
}

/**
 * @brief 在启动外设前校验唯一的生产配置来源。
 *
 * 任一项非法时直接进入 Error_Handler，模块不会再用默认值或夹紧值替代。
 */
static uint8_t App_IsValidConfig(void)
{
    if ((!isfinite(APP_CONTROL_DT_S) ||
         (APP_CONTROL_DT_S <= 0.0f)) ||
        (!isfinite(APP_RAMP_MAX_SPEED_DEG_S) ||
         (APP_RAMP_MAX_SPEED_DEG_S <= 0.0f)) ||
        (!isfinite(APP_RAMP_MAX_ACCEL_DEG_S2) ||
         (APP_RAMP_MAX_ACCEL_DEG_S2 <= 0.0f)) ||
        (APP_SELF_TEST_MS == 0U) ||
        (APP_TELEMETRY_MS == 0U) ||
        (APP_LED_UPDATE_MS == 0U) ||
        (APP_DEBUG_HEARTBEAT_MS == 0U))
    {
        return 0U;
    }

    if ((g_gm6020_config.id < 1U) ||
        (g_gm6020_config.id > 7U) ||
        (g_gm6020_config.zero_raw > 0x1FFFU) ||
        ((g_gm6020_config.angle_sign != 1) &&
         (g_gm6020_config.angle_sign != -1)) ||
        ((g_gm6020_config.voltage_sign != 1) &&
         (g_gm6020_config.voltage_sign != -1)) ||
        (!isfinite(g_gm6020_config.gear_ratio) ||
         (g_gm6020_config.gear_ratio <= 0.0f)) ||
        (!isfinite(g_gm6020_config.max_speed_rpm) ||
         (g_gm6020_config.max_speed_rpm <= 0.0f)) ||
        (g_gm6020_config.voltage_limit_raw <= 0) ||
        (g_gm6020_config.voltage_limit_raw >
         GM6020_MAX_VOLTAGE_RAW) ||
        (g_gm6020_config.online_timeout_ms == 0U) ||
        !App_IsValidPidConfig(&g_gm6020_config.position) ||
        !App_IsValidPidConfig(&g_gm6020_config.speed))
    {
        return 0U;
    }

    if ((g_vesc_config.controller_id > 0xFFU) ||
        (g_vesc_config.online_timeout_ms == 0U) ||
        (!isfinite(APP_MASTER_ANGLE_SIGN) ||
         (fabsf(APP_MASTER_ANGLE_SIGN) != 1.0f)) ||
        !isfinite(APP_MASTER_ANGLE_OFFSET_DEG))
    {
        return 0U;
    }

#if WHEEL_BUILD_DEBUG
    if ((g_wheel_debug_config.use_ramp > 1U) ||
        (g_wheel_debug_config.erpm_limit < 0) ||
        (g_wheel_debug_config.min_period_ms == 0U) ||
        (g_wheel_debug_config.wave_period_ms <
         g_wheel_debug_config.min_period_ms) ||
        !isfinite(g_wheel_debug_config.max_amplitude_deg) ||
        (g_wheel_debug_config.max_amplitude_deg <= 0.0f) ||
        !isfinite(g_wheel_debug_config.wave_amplitude_deg) ||
        (fabsf(g_wheel_debug_config.wave_amplitude_deg) >
         g_wheel_debug_config.max_amplitude_deg) ||
        !isfinite(g_wheel_debug_config.target_angle_deg) ||
        !isfinite(g_wheel_debug_config.target_speed_rpm) ||
        (fabsf(g_wheel_debug_config.target_speed_rpm) >
         g_gm6020_config.max_speed_rpm) ||
        (g_wheel_debug_config.target_voltage_raw >
         g_gm6020_config.voltage_limit_raw) ||
        (g_wheel_debug_config.target_voltage_raw <
         -g_gm6020_config.voltage_limit_raw))
    {
        return 0U;
    }
#else
    if ((g_master_config.wheel_id < 1U) ||
        (g_master_config.wheel_id > 4U) ||
        (g_master_config.link_timeout_ms == 0U))
    {
        return 0U;
    }
#endif

    return 1U;
}

#if !WHEEL_BUILD_DEBUG
/**
 * @brief 计算串口显示用的事件年龄。
 * @return 时间戳为 0 时返回 0xFFFFFFFF，表示事件尚未发生。
 */
static uint32_t App_AgeMs(uint32_t now_ms,
                          uint32_t timestamp_ms)
{
    if (timestamp_ms == 0U)
    {
        return 0xFFFFFFFFU;
    }

    return (uint32_t)(now_ms - timestamp_ms);
}
#endif

#if !WHEEL_BUILD_DEBUG
/** @brief 将任意角度归一化到 [0, 360)。 */
static float App_Wrap360(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}
#endif

#if WHEEL_BUILD_DEBUG
/** @brief 将值限制在 [-limit, +limit]；limit 非正时返回 0。 */
static float App_ClampAbs(float value, float limit)
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
#endif

/**
 * @brief 同时失能 GM6020 和 VESC。
 *
 * 只有当前处于使能状态的驱动才调用 Disable，避免每毫秒重复发送失能帧。
 * 驱动自身的 ControlStep 在未使能时仍会持续发送零输出。
 */
static void App_DisableActuators(App_t *app)
{
    if (app->gm6020.is_enabled)
    {
        GM6020_Disable(&app->gm6020);
    }
    if (app->vesc.is_enabled)
    {
        VESC_Disable(&app->vesc);
    }

    app->wheel_active = 0U;
}

/**
 * @brief 根据最近反馈时间刷新两个执行器的应用层在线状态。
 *
 * 驱动对象中的 is_online 表示“曾经收到过反馈”，这里进一步比较
 * last_rx_ms 和超时窗口，使 LED、安全判断和调试使能及时响应断线。
 */
static void App_UpdateOnlineState(App_t *app)
{
    app->gm6020_online =
        (app->gm6020.is_online &&
         (App_ElapsedMs(app->ms,
                        app->gm6020.last_rx_ms) <=
          app->gm6020.online_timeout_ms)) ? 1U : 0U;
    app->vesc_online =
        (app->vesc.is_online &&
         (App_ElapsedMs(app->ms,
                        app->vesc.last_rx_ms) <=
          app->vesc.online_timeout_ms)) ? 1U : 0U;
}

#if !WHEEL_BUILD_DEBUG
/**
 * @brief 进入正式模式的整轮失能状态。
 *
 * need_fresh_control 被置位后，即使主控链路和设备恢复，也必须再收到一帧
 * 新控制命令才能重新运动，防止执行故障前保存的旧目标。
 */
static void App_DisableWheel(App_t *app)
{
    App_DisableActuators(app);
    app->need_fresh_control = 1U;
}

/**
 * @brief 应用一帧新的主控控制命令。
 *
 * 处理顺序：
 * 1. 主控角度按 sign/offset 映射到舵轮本地坐标；
 * 2. AngleOpt 选择更近的物理等价方向；
 * 3. 首次活动时使能两个执行器，并把斜坡起点设为当前实际角；
 * 4. 下发 GM6020 最终位置目标和 VESC ERPM；
 * 5. 清除 pending 标志，避免同一命令重复应用。
 *
 * @note 只允许在 TIM6 1 kHz 上下文中调用。
 */
static void App_ApplyPendingControl(App_t *app)
{
    AngleOptResult_t result;
    float mapped_target_deg;

    mapped_target_deg = App_Wrap360(
        (APP_MASTER_ANGLE_SIGN * app->master.target_angle_deg) +
        APP_MASTER_ANGLE_OFFSET_DEG);

    AngleOpt_Select(app->gm6020.angle_deg,
                    mapped_target_deg,
                    app->master.drive_erpm,
                    &result);

    if (!app->wheel_active)
    {
        GM6020_Enable(&app->gm6020);
        VESC_Enable(&app->vesc);
        SetpointRamp_Reset(&app->target_ramp,
                           app->gm6020.angle_deg);
        app->wheel_active = 1U;
    }

    app->mapped_target_deg = mapped_target_deg;
    app->final_target_angle_deg = result.target_angle_deg;
    app->final_target_erpm = result.target_erpm;
    app->angle_reversed = result.reversed;

    SetpointRamp_SetDesired(&app->target_ramp,
                            result.target_angle_deg);
    VESC_SetTargetErpm(&app->vesc, result.target_erpm);

    app->control_pending = 0U;
    app->need_fresh_control = 0U;
}

/**
 * @brief 正式固件的 1 kHz 控制步骤。
 *
 * 控制条件采用“全有或全无”策略：主控链路、控制命令、GM6020 和 VESC
 * 必须同时可用，否则整轮失能。故障期间清除 pending，恢复后要求新命令。
 */
static void App_ProductionControlStep(App_t *app)
{
    uint8_t master_ready;
    uint8_t actuators_online;

    MasterCan_Update(&app->master, app->ms);
    App_UpdateOnlineState(app);

    if (app->ms < APP_SELF_TEST_MS)
    {
        App_DisableWheel(app);
        GM6020_ControlStep(&app->gm6020, app->ms);
        VESC_ControlStep(&app->vesc, app->ms);
        return;
    }

    /* 心跳维持链路，control_seen 表示本链路周期内收到过目标。 */
    master_ready =
        (app->master.link_valid &&
         app->master.control_seen) ? 1U : 0U;
    actuators_online =
        (app->gm6020_online &&
         app->vesc_online) ? 1U : 0U;

    /*
     * 任一必要条件失效时整轮闭合。清除 pending 是故障恢复安全语义的
     * 关键：旧命令不能在链路恢复后被自动补执行。
     */
    if (!master_ready || !actuators_online)
    {
        app->control_pending = 0U;
        App_DisableWheel(app);
    }
    else
    {
        if (app->control_pending)
        {
            App_ApplyPendingControl(app);
        }

        /* 活动期间只推进位置斜坡；VESC 目标由主控帧直接更新。 */
        if (app->wheel_active &&
            !app->need_fresh_control)
        {
            SetpointRamp_Update(&app->target_ramp,
                                APP_CONTROL_DT_S);
            GM6020_SetTargetAngle(
                &app->gm6020,
                SetpointRamp_GetOutput(&app->target_ramp));
        }
    }

    GM6020_ControlStep(&app->gm6020, app->ms);
    VESC_ControlStep(&app->vesc, app->ms);
}
#else
/** @brief 判断调试模式是否使用任一 GM6020 控制入口。 */
static uint8_t App_DebugUsesGm(WheelDebugMode_t mode)
{
    return (WheelDebug_ModeUsesGmPosition(mode) ||
            WheelDebug_ModeUsesGmSpeed(mode) ||
            WheelDebug_ModeUsesGmVoltage(mode)) ? 1U : 0U;
}

/**
 * @brief 调试模式的统一安全停机。
 *
 * 模式切换、设备掉线、自检阶段和 DISABLED 模式都走这里。除了失能执行器，
 * 同时清除上一模式留下的目标诊断值，避免串口显示与实际运行不符。
 */
static void App_DebugClearEnable(App_t *app)
{
    g_wheel_debug.enable = 0U;
    WheelDebug_ResetRuntime(&app->debug_runtime);
    App_DisableActuators(app);
    app->angle_reversed = 0U;
    app->mapped_target_deg = 0.0f;
    app->final_target_angle_deg = 0.0f;
    app->final_target_erpm = 0;
}

/**
 * @brief 调试固件的 1 kHz 控制步骤。
 *
 * 本函数先让 WheelDebug 生成目标，再根据模式只使能所需设备。其它设备
 * 始终保持零输出，因此 GM6020 调参时不会带着旧 VESC 目标运行。
 *
 * 安全顺序固定为：在线判断 -> 模式变化处理 -> 显式使能检查 -> 目标下发
 * -> 驱动 ControlStep。任何一步失败都会清除调试使能并要求人工重启。
 */
static void App_DebugControlStep(App_t *app)
{
    WheelDebugMode_t mode;
    uint8_t debug_enable;
    int32_t erpm_limit;
    int32_t target_erpm;

    App_UpdateOnlineState(app);
    WheelDebug_Update(&g_wheel_debug,
                      &app->debug_runtime,
                      &app->target_ramp,
                      app->gm6020.angle_deg,
                      APP_CONTROL_DT_S);
    debug_enable = g_wheel_debug.enable;

    /* 模式变化可能改变控制层级，必须先停止所有执行器。 */
    if (app->debug_runtime.mode_changed)
    {
        App_DebugClearEnable(app);
    }

    mode = app->debug_runtime.previous_mode;
    if (app->ms < APP_SELF_TEST_MS)
    {
        App_DebugClearEnable(app);
        GM6020_ControlStep(&app->gm6020, app->ms);
        VESC_ControlStep(&app->vesc, app->ms);
        return;
    }

    /* 当前模式只需要目标设备在线，不要求另一台设备同时连接。 */
    if (App_DebugUsesGm(mode) && !app->gm6020_online)
    {
        App_DebugClearEnable(app);
    }
    if (WheelDebug_ModeUsesVesc(mode) && !app->vesc_online)
    {
        App_DebugClearEnable(app);
    }

    switch (mode)
    {
        case WHEEL_DEBUG_GM_POSITION_STEP:
        case WHEEL_DEBUG_GM_POSITION_SINE:
            /*
             * 两种位置模式共用同一位置环。阶跃和正弦的差别已在
             * WheelDebug_Update() 中转换成 ramp_output_angle_deg。
             */
            GM6020_SetControlMode(&app->gm6020,
                                  GM6020_CONTROL_POSITION);
            VESC_SetTargetErpm(&app->vesc, 0);
            if (app->vesc.is_enabled)
            {
                VESC_Disable(&app->vesc);
            }

            if ((debug_enable != 0U) &&
                app->gm6020_online)
            {
                if (!app->gm6020.is_enabled)
                {
                    GM6020_Enable(&app->gm6020);
                }
                GM6020_SetTargetAngle(
                    &app->gm6020,
                    app->debug_runtime.ramp_output_angle_deg);

                app->mapped_target_deg =
                    app->debug_runtime.source_angle_deg;
                app->final_target_angle_deg =
                    app->debug_runtime.angle_result.target_angle_deg;
                app->final_target_erpm =
                    app->debug_runtime.angle_result.target_erpm;
                app->angle_reversed =
                    app->debug_runtime.angle_result.reversed;
                app->wheel_active =
                    app->gm6020.is_enabled;
            }
            else
            {
                if (app->gm6020.is_enabled)
                {
                    GM6020_Disable(&app->gm6020);
                }
                app->wheel_active = 0U;
            }
            break;

        case WHEEL_DEBUG_GM_SPEED:
            /* 速度模式只使用 GM6020 内环，VESC 保持零输出。 */
            GM6020_SetControlMode(&app->gm6020,
                                  GM6020_CONTROL_SPEED);
            VESC_SetTargetErpm(&app->vesc, 0);
            if (app->vesc.is_enabled)
            {
                VESC_Disable(&app->vesc);
            }

            if ((debug_enable != 0U) &&
                app->gm6020_online)
            {
                if (!app->gm6020.is_enabled)
                {
                    GM6020_Enable(&app->gm6020);
                }
                GM6020_SetTargetSpeed(
                    &app->gm6020,
                    g_wheel_debug.target_speed_rpm);
                app->final_target_erpm = 0;
                app->wheel_active =
                    app->gm6020.is_enabled;
            }
            else
            {
                if (app->gm6020.is_enabled)
                {
                    GM6020_Disable(&app->gm6020);
                }
                app->wheel_active = 0U;
            }
            break;

        case WHEEL_DEBUG_GM_VOLTAGE:
            /* 直接电压绕过两个 PID，但仍经过驱动内电压限幅。 */
            GM6020_SetControlMode(&app->gm6020,
                                  GM6020_CONTROL_VOLTAGE);
            VESC_SetTargetErpm(&app->vesc, 0);
            if (app->vesc.is_enabled)
            {
                VESC_Disable(&app->vesc);
            }

            if ((debug_enable != 0U) &&
                app->gm6020_online)
            {
                if (!app->gm6020.is_enabled)
                {
                    GM6020_Enable(&app->gm6020);
                }
                GM6020_SetTargetVoltage(
                    &app->gm6020,
                    g_wheel_debug.target_voltage_raw);
                app->final_target_erpm = 0;
                app->wheel_active =
                    app->gm6020.is_enabled;
            }
            else
            {
                if (app->gm6020.is_enabled)
                {
                    GM6020_Disable(&app->gm6020);
                }
                app->wheel_active = 0U;
            }
            break;

        case WHEEL_DEBUG_VESC_SPEED:
            /* VESC 模式只验证本地 CAN 通信，GM6020 保持失能。 */
            if (app->gm6020.is_enabled)
            {
                GM6020_Disable(&app->gm6020);
            }

            erpm_limit = g_wheel_debug.erpm_limit;
            if (erpm_limit < 0)
            {
                erpm_limit = -erpm_limit;
            }

            if ((debug_enable != 0U) &&
                app->vesc_online)
            {
                if (!app->vesc.is_enabled)
                {
                    VESC_Enable(&app->vesc);
                }

                target_erpm =
                    (int32_t)App_ClampAbs(
                        (float)g_wheel_debug.target_erpm,
                        (float)erpm_limit);
                VESC_SetTargetErpm(&app->vesc, target_erpm);
                app->final_target_erpm = target_erpm;
                app->wheel_active =
                    app->vesc.is_enabled;
            }
            else
            {
                VESC_SetTargetErpm(&app->vesc, 0);
                if (app->vesc.is_enabled)
                {
                    VESC_Disable(&app->vesc);
                }
                app->wheel_active = 0U;
            }
            break;

        case WHEEL_DEBUG_DISABLED:
        default:
            /* 非法 Watch 值在 WheelDebug 中已回退为 DISABLED。 */
            App_DebugClearEnable(app);
            break;
    }

    GM6020_ControlStep(&app->gm6020, app->ms);
    VESC_ControlStep(&app->vesc, app->ms);
}
#endif

/** @brief 根据编译宏分派正式或调试控制路径。 */
static void App_ControlStep(App_t *app)
{
#if WHEEL_BUILD_DEBUG
    App_DebugControlStep(app);
#else
    App_ProductionControlStep(app);
#endif
}

/**
 * @brief 组装串口遥测中的故障位。
 *
 * 正式模式报告主控、两个执行器和恢复条件；调试模式只报告当前模式
 * 实际使用设备的在线及发送故障，避免未连接的备用设备产生误导信息。
 */
static uint32_t App_GetFaultFlags(const App_t *app)
{
    uint32_t flags = 0U;

#if WHEEL_BUILD_DEBUG
    WheelDebugMode_t mode = g_wheel_debug.mode;

    if (App_DebugUsesGm(mode) &&
        !app->gm6020_online)
    {
        flags |= APP_FAULT_GM6020_OFFLINE;
    }
    if (WheelDebug_ModeUsesVesc(mode) &&
        !app->vesc_online)
    {
        flags |= APP_FAULT_VESC_OFFLINE;
    }
    if ((App_DebugUsesGm(mode)) &&
        (app->gm6020.last_tx_status != HAL_OK))
    {
        flags |= APP_FAULT_GM6020_TX;
    }
    if (WheelDebug_ModeUsesVesc(mode) &&
        (app->vesc.last_tx_status != HAL_OK))
    {
        flags |= APP_FAULT_VESC_TX;
    }
#else
    if (!app->master.link_valid)
    {
        flags |= APP_FAULT_MASTER_LINK;
    }
    if (!app->master.control_seen)
    {
        flags |= APP_FAULT_NO_CONTROL;
    }
    if (!app->gm6020_online)
    {
        flags |= APP_FAULT_GM6020_OFFLINE;
    }
    if (!app->vesc_online)
    {
        flags |= APP_FAULT_VESC_OFFLINE;
    }
    if (app->need_fresh_control)
    {
        flags |= APP_FAULT_NEED_FRESH;
    }
    if (app->gm6020.last_tx_status != HAL_OK)
    {
        flags |= APP_FAULT_GM6020_TX;
    }
    if (app->vesc.last_tx_status != HAL_OK)
    {
        flags |= APP_FAULT_VESC_TX;
    }
#endif

    return flags;
}

/**
 * @brief 刷新四个状态 LED。
 *
 * 所有 LED 均为低电平点亮。正式模式下 PB4 表示主控链路；调试模式下
 * PB4 变为 1 Hz 心跳。PB5/PB7 始终显示 VESC/GM6020 在线状态，PB6
 * 显示当前调试模式所选执行器是否使能。
 */
static void App_UpdateLeds(App_t *app)
{
    uint8_t enabled;

    if (app->ms < APP_SELF_TEST_MS)
    {
        HAL_GPIO_WritePin(
            GPIOB,
            APP_LED_MASTER_PIN |
                APP_LED_VESC_PIN |
                APP_LED_ENABLE_PIN |
                APP_LED_GM6020_PIN,
            GPIO_PIN_RESET);
        return;
    }

#if WHEEL_BUILD_DEBUG
    {
        uint8_t heartbeat =
            (((app->ms / APP_DEBUG_HEARTBEAT_MS) % 2U) == 0U)
                ? 1U : 0U;
        WheelDebugMode_t mode = g_wheel_debug.mode;

        if (WheelDebug_ModeUsesVesc(mode))
        {
            enabled = app->vesc.is_enabled;
        }
        else if (App_DebugUsesGm(mode))
        {
            enabled = app->gm6020.is_enabled;
        }
        else
        {
            enabled = 0U;
        }

        HAL_GPIO_WritePin(GPIOB,
                          APP_LED_MASTER_PIN,
                          heartbeat ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
#else
    enabled =
        (app->wheel_active &&
         app->gm6020.is_enabled &&
         app->vesc.is_enabled) ? 1U : 0U;

    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_MASTER_PIN,
                      app->master.link_valid
                          ? GPIO_PIN_RESET
                          : GPIO_PIN_SET);
#endif

    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_VESC_PIN,
                      app->vesc_online
                          ? GPIO_PIN_RESET
                          : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_ENABLE_PIN,
                      enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_GM6020_PIN,
                      app->gm6020_online
                          ? GPIO_PIN_RESET
                          : GPIO_PIN_SET);
}

/**
 * @brief 组装并以非阻塞方式发送 50 Hz CSV 遥测。
 *
 * 前 27 列在正式和调试固件中保持完全相同的顺序。调试固件在末尾追加
 * 调试模式、目标、斜坡和 PID 中间项，便于使用 VOFA+ 等工具记录响应。
 * UART 忙时直接跳过本次发送，绝不阻塞 1 kHz 控制中断。
 */
static void App_SendTelemetry(App_t *app)
{
    uint8_t master_link = 0U;
    uint8_t control_seen = 0U;
    uint8_t need_fresh = 0U;
    uint32_t control_age_ms;
    uint32_t heartbeat_age_ms;
    int length;

#if !WHEEL_BUILD_DEBUG
    master_link = app->master.link_valid;
    control_seen = app->master.control_seen;
    need_fresh = app->need_fresh_control;
    control_age_ms =
        App_AgeMs(app->ms, app->master.last_control_ms);
    heartbeat_age_ms =
        App_AgeMs(app->ms, app->master.last_heartbeat_ms);
#else
    control_age_ms = 0xFFFFFFFFU;
    heartbeat_age_ms = 0xFFFFFFFFU;
#endif

    if (app->uart_busy)
    {
        return;
    }

    /*
     * 正式版固定 27 列：
     * 1 时间, 2 主控链路, 3 control_seen, 4 GM在线, 5 VESC在线,
     * 6 整轮活动, 7 方向反转, 8 需新命令, 9 主控ERPM, 10 角度码,
     * 11 主控目标角, 12 映射目标角, 13 最终舵向角, 14 最终ERPM,
     * 15 GM实际角, 16 GM转速, 17 GM电流, 18 GM温度, 19 GM电压,
     * 20 VESC实际ERPM, 21 VESC电流, 22 VESC占空比, 23 斜坡角,
     * 24 斜坡速度, 25 控制帧年龄, 26 心跳帧年龄, 27 故障位。
     * 调试版将前 27 列中的主控字段填零，保持列序不变。
     */
    length = snprintf(
        app->uart_buffer,
        sizeof(app->uart_buffer),
        "%u,%u,%u,%u,%u,%u,%u,%u,"
        "%d,%u,%d,%d,%d,%d,"
        "%d,%d,%d,%u,%d,"
        "%d,%d,%d,%d,%d,"
        "%u,%u,%u",
        (unsigned)app->ms,
        (unsigned)master_link,
        (unsigned)control_seen,
        (unsigned)app->gm6020_online,
        (unsigned)app->vesc_online,
        (unsigned)app->wheel_active,
        (unsigned)app->angle_reversed,
        (unsigned)need_fresh,
#if WHEEL_BUILD_DEBUG
        0,
#else
        (int)app->master.drive_erpm,
#endif
#if WHEEL_BUILD_DEBUG
        0U,
#else
        (unsigned)app->master.angle_code,
#endif
#if WHEEL_BUILD_DEBUG
        0,
#else
        (int)(app->master.target_angle_deg * 100.0f),
#endif
        (int)(app->mapped_target_deg * 100.0f),
        (int)(app->final_target_angle_deg * 100.0f),
        (int)app->final_target_erpm,
        (int)(app->gm6020.angle_deg * 100.0f),
        (int)app->gm6020.speed_rpm,
        (int)app->gm6020.current_raw,
        (unsigned)app->gm6020.temperature,
        (int)app->gm6020.target_voltage,
        (int)app->vesc.actual_erpm,
        (int)(app->vesc.actual_current_a * 1000.0f),
        (int)(app->vesc.duty * 1000.0f),
        (int)(SetpointRamp_GetOutput(&app->target_ramp) *
              100.0f),
        (int)SetpointRamp_GetVelocity(&app->target_ramp),
        (unsigned)control_age_ms,
        (unsigned)heartbeat_age_ms,
        (unsigned)App_GetFaultFlags(app));

#if !WHEEL_BUILD_DEBUG
    /* 正式固件严格保持原有 27 列格式，仅在此追加行结束符。 */
    if ((length > 0) &&
        ((uint32_t)length < sizeof(app->uart_buffer)))
    {
        int ending_length = snprintf(
            app->uart_buffer + length,
            sizeof(app->uart_buffer) - (uint32_t)length,
            "\r\n");

        if ((ending_length <= 0) ||
            ((uint32_t)ending_length >=
             (sizeof(app->uart_buffer) - (uint32_t)length)))
        {
            return;
        }
        length += ending_length;
    }
#endif

#if WHEEL_BUILD_DEBUG
    /*
     * 第 28..44 列：模式、使能、位置输入、方向ERPM、波形源角、
     * 斜坡开关、GM目标RPM、位置P/I/D、速度P/I/D和CAN1诊断。
     */
    if ((length > 0) &&
        ((uint32_t)length < sizeof(app->uart_buffer)))
    {
        int extra_length = snprintf(
            app->uart_buffer + length,
            sizeof(app->uart_buffer) - (uint32_t)length,
            ",%u,%u,%d,%d,%d,%u,%d,"
            "%d,%d,%d,%d,%d,%d,"
            "%u,%u,%u,%d\r\n",
            (unsigned)g_wheel_debug.mode,
            (unsigned)g_wheel_debug.enable,
            (int)(g_wheel_debug.target_angle_deg * 100.0f),
            (int)g_wheel_debug.direction_erpm,
            (int)(app->debug_runtime.source_angle_deg * 100.0f),
            (unsigned)g_wheel_debug.use_ramp,
            (int)app->gm6020.target_speed_rpm,
            (int)(app->gm6020.position_pid.proportional * 100.0f),
            (int)(app->gm6020.position_pid.integral * 100.0f),
            (int)(app->gm6020.position_pid.differential * 100.0f),
            (int)(app->gm6020.speed_pid.proportional * 100.0f),
            (int)(app->gm6020.speed_pid.integral * 100.0f),
            (int)(app->gm6020.speed_pid.differential * 100.0f),
            (unsigned)HAL_CAN_GetState(app->hcan1),
            (unsigned)HAL_CAN_GetError(app->hcan1),
            (unsigned)HAL_CAN_GetTxMailboxesFreeLevel(
                app->hcan1),
            (int)g_wheel_debug.erpm_limit);

        if ((extra_length <= 0) ||
            ((uint32_t)extra_length >=
             (sizeof(app->uart_buffer) - (uint32_t)length)))
        {
            return;
        }
        length += extra_length;
    }
#endif

    if ((length > 0) &&
        ((uint32_t)length < sizeof(app->uart_buffer)))
    {
        if (HAL_UART_Transmit_IT(
                app->huart1,
                (uint8_t *)app->uart_buffer,
                (uint16_t)length) == HAL_OK)
        {
            app->uart_busy = 1U;
        }
    }
}

/**
 * @brief 初始化应用对象和编译期选择的输入路径。
 *
 * @param app 待初始化的应用上下文。
 * @param hcan1 本地电机 CAN1。
 * @param hcan2 主控 CAN2；仅正式构建要求非空。
 * @param htim6 1 kHz 控制定时器。
 * @param huart1 调试遥测串口。
 *
 * 本函数不启动 CAN 或定时器，确保所有对象和过滤器准备完成后再进入
 * 实时控制阶段。
 */
void App_Init(App_t *app,
              CAN_HandleTypeDef *hcan1,
              CAN_HandleTypeDef *hcan2,
              TIM_HandleTypeDef *htim6,
              UART_HandleTypeDef *huart1)
{
    if ((app == NULL) || (hcan1 == NULL) ||
        (htim6 == NULL) || (huart1 == NULL))
    {
        Error_Handler();
        return;
    }

    if (!App_IsValidConfig())
    {
        Error_Handler();
        return;
    }

#if !WHEEL_BUILD_DEBUG
    if (hcan2 == NULL)
    {
        Error_Handler();
        return;
    }
#else
    (void)hcan2;
#endif

    /* 先清空整个对象，防止未显式赋值的状态带着随机内容上电。 */
    memset(app, 0, sizeof(*app));
    app->hcan1 = hcan1;
#if !WHEEL_BUILD_DEBUG
    app->hcan2 = hcan2;
#endif
    app->htim6 = htim6;
    app->huart1 = huart1;
    app->need_fresh_control = 1U;

    /* 驱动只接收一次完整配置，不在初始化后再重建控制器。 */
    GM6020_Init(&app->gm6020,
                hcan1,
                &g_gm6020_config);

    /* VESC 内部闭环由控制器负责，分控板只配置 ID 和通讯超时。 */
    VESC_Init(&app->vesc, hcan1, &g_vesc_config);
    SetpointRamp_Init(&app->target_ramp,
                      0.0f,
                      &g_ramp_config);

#if WHEEL_BUILD_DEBUG
    WheelDebug_Init(&g_wheel_debug,
                    &app->debug_runtime,
                    &g_wheel_debug_config);
#else
    MasterCan_Init(&app->master, &g_master_config);
#endif

    /* 串口中断优先级低于 1 kHz 控制中断，仅负责非阻塞发送。 */
    HAL_NVIC_SetPriority(USART1_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief 启动 CAN 接收和电机控制节拍。
 *
 * CAN1 始终接收本地电机总线；CAN2 仅在正式固件中启动，并使用本轮
 * 控制帧和心跳帧过滤器。TIM6 最后启动，确保进入第一次控制中断前，
 * 驱动对象、CAN 和串口均已经可用。
 */
void App_Start(App_t *app)
{
    if (app == NULL)
    {
        return;
    }

    if (BSP_CAN_StartAcceptAll(app->hcan1) != HAL_OK)
    {
        Error_Handler();
        return;
    }

#if !WHEEL_BUILD_DEBUG
    if (BSP_CAN_StartMasterFilter(
            app->hcan2,
            app->master.control_id,
            app->master.heartbeat_id) != HAL_OK)
    {
        Error_Handler();
        return;
    }
#endif

    if (HAL_TIM_Base_Start_IT(app->htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 主循环的低频任务调度。
 *
 * 使用 TIM6 的毫秒时钟计算时间差，因此主循环执行频率变化时，LED 和
 * 遥测周期仍保持稳定。电机控制不在这里执行。
 */
void App_Loop(App_t *app)
{
    if (app == NULL)
    {
        return;
    }

    if ((uint32_t)(app->ms - app->last_led_ms) >=
        APP_LED_UPDATE_MS)
    {
        app->last_led_ms = app->ms;
        App_UpdateLeds(app);
    }

    if ((uint32_t)(app->ms - app->last_telemetry_ms) >=
        APP_TELEMETRY_MS)
    {
        app->last_telemetry_ms = app->ms;
        App_SendTelemetry(app);
    }
}

/**
 * @brief 处理一次 CAN RX FIFO0 挂起事件。
 *
 * 函数会排空当前 FIFO。CAN1 的每一帧同时交给 GM6020 和 VESC 解码，
 * 两个驱动会拒绝不属于自己的 ID。正式固件的 CAN2 帧交给 MasterCan；
 * 只有格式正确的控制帧才设置 control_pending。
 */
void App_OnCanRx(App_t *app, CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if ((app == NULL) || (hcan == NULL))
    {
        return;
    }

    if (hcan == app->hcan1)
    {
        while (HAL_CAN_GetRxFifoFillLevel(
                   hcan,
                   CAN_RX_FIFO0) > 0U)
        {
            if (HAL_CAN_GetRxMessage(
                    hcan,
                    CAN_RX_FIFO0,
                    &header,
                    data) != HAL_OK)
            {
                break;
            }

            GM6020_OnRx(&app->gm6020,
                        &header,
                        data,
                        app->ms);
            VESC_OnRx(&app->vesc,
                      &header,
                      data,
                      app->ms);
        }
        return;
    }

#if !WHEEL_BUILD_DEBUG
    if (hcan == app->hcan2)
    {
        while (HAL_CAN_GetRxFifoFillLevel(
                   hcan,
                   CAN_RX_FIFO0) > 0U)
        {
            uint8_t valid_master_control;

            if (HAL_CAN_GetRxMessage(
                    hcan,
                    CAN_RX_FIFO0,
                    &header,
                    data) != HAL_OK)
            {
                break;
            }

            valid_master_control =
                ((header.IDE == CAN_ID_STD) &&
                 (header.RTR == CAN_RTR_DATA) &&
                 (header.DLC == 8U) &&
                 (header.StdId ==
                  app->master.control_id)) ? 1U : 0U;

            MasterCan_OnRx(&app->master,
                           &header,
                           data,
                           app->ms);

            if (valid_master_control)
            {
                app->control_pending = 1U;
            }
        }
    }
#endif
}

/**
 * @brief TIM6 周期中断入口。
 *
 * 先推进 32 位毫秒时钟，再执行一次完整控制步骤。其它定时器实例不会
 * 影响舵轮状态。
 */
void App_OnTimerElapsed(App_t *app, TIM_HandleTypeDef *htim)
{
    if ((app == NULL) || (htim == NULL) ||
        (htim->Instance != app->htim6->Instance))
    {
        return;
    }

    app->ms++;
    App_ControlStep(app);
}

/** @brief UART1 发送完成回调，允许下一次遥测覆盖发送缓冲。 */
void App_OnUartTxComplete(App_t *app, UART_HandleTypeDef *huart)
{
    if ((app != NULL) && (huart == app->huart1))
    {
        app->uart_busy = 0U;
    }
}
