/**
 * @file WheelDebug.c
 * @brief 将 Watch 输入转换为位置、速度和 VESC 调试目标。
 *
 * 数据流：
 *   g_wheel_debug
 *     -> 模式合法性检查和模式变化检测
 *     -> 阶跃/正弦位置生成
 *     -> AngleOpt 选择等价方向
 *     -> 可选 SetpointRamp
 *     -> App 选择 GM6020 或 VESC 控制入口
 *
 * 本文件不访问 CAN、GPIO 或电机，因此可以在主机端单独测试。
 */

#include "WheelDebug.h"

#include <math.h>
#include <stddef.h>

#include "AppConfig.h"

/* 调试波形硬限制，与 APP_DEBUG_DEFAULT_* 默认值相互独立。 */
#define WHEEL_DEBUG_TWO_PI 6.283185307179586f
#define WHEEL_DEBUG_MIN_PERIOD_MS 200U
#define WHEEL_DEBUG_MAX_AMPLITUDE_DEG 180.0f

/* Watch 修改此对象；App 在 TIM6 控制步骤中读取。 */
WheelDebug_t g_wheel_debug;

/** @brief 检查 Watch 写入的模式是否属于已定义枚举范围。 */
static uint8_t WheelDebug_IsValidMode(WheelDebugMode_t mode)
{
    return ((mode >= WHEEL_DEBUG_DISABLED) &&
            (mode <= WHEEL_DEBUG_VESC_SPEED)) ? 1U : 0U;
}

/** @brief 把数值限制在 [-limit, +limit]；非正 limit 返回 0。 */
static float WheelDebug_ClampAbs(float value, float limit)
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

/** @brief 把角度归一化到 [0, 360)。 */
static float WheelDebug_Wrap360(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/** @brief 判断模式是否生成 GM6020 位置目标。 */
static uint8_t WheelDebug_IsPositionMode(WheelDebugMode_t mode)
{
    return ((mode == WHEEL_DEBUG_GM_POSITION_STEP) ||
            (mode == WHEEL_DEBUG_GM_POSITION_SINE)) ? 1U : 0U;
}

/**
 * @brief 清空位置调试中间状态。
 *
 * 模式切换、断线或 App 主动停机会调用此函数。复位后下一次重新使能会
 * 从当时的实际角重新建立正弦原点和斜坡起点，避免沿用失联前状态。
 */
void WheelDebug_ResetRuntime(WheelDebugRuntime_t *runtime)
{
    if (runtime == NULL)
    {
        return;
    }

    SetpointRamp_Init(&runtime->ramp,
                      0.0f,
                      SETPOINT_RAMP_DEFAULT_MAX_SPEED_DEG_S,
                      SETPOINT_RAMP_DEFAULT_MAX_ACCEL_DEG_S2);
    runtime->angle_result.target_angle_deg = 0.0f;
    runtime->angle_result.target_erpm = 0;
    runtime->angle_result.reversed = 0U;
    runtime->source_angle_deg = 0.0f;
    runtime->ramp_output_angle_deg = 0.0f;
    runtime->sine_origin_deg = 0.0f;
    runtime->sine_phase = 0.0f;
    runtime->initialized = 0U;
    runtime->ramp_active = 0U;
}

/**
 * @brief 恢复调试 Watch 默认值和安全初态。
 *
 * 初始模式固定为 DISABLED、使能为 0。目标值来自 AppConfig.h，使上电
 * 后即使误操作到其他模式，也不会自动产生运动。
 */
void WheelDebug_Init(WheelDebug_t *debug,
                     WheelDebugRuntime_t *runtime)
{
    if ((debug == NULL) || (runtime == NULL))
    {
        return;
    }

    debug->mode = WHEEL_DEBUG_DISABLED;
    debug->enable = 0U;
    debug->target_angle_deg = APP_DEBUG_DEFAULT_ANGLE_DEG;
    debug->direction_erpm = APP_DEBUG_DEFAULT_DIRECTION_ERPM;
    debug->target_speed_rpm = APP_DEBUG_DEFAULT_SPEED_RPM;
    debug->target_voltage_raw = APP_DEBUG_DEFAULT_VOLTAGE_RAW;
    debug->target_erpm = APP_DEBUG_DEFAULT_TARGET_ERPM;
    debug->erpm_limit = APP_DEBUG_DEFAULT_ERPM_LIMIT;
    debug->use_ramp = APP_DEBUG_DEFAULT_USE_RAMP;
    debug->wave_amplitude_deg = APP_DEBUG_DEFAULT_AMPLITUDE_DEG;
    debug->wave_period_ms = APP_DEBUG_DEFAULT_PERIOD_MS;

    WheelDebug_ResetRuntime(runtime);
    runtime->previous_mode = WHEEL_DEBUG_DISABLED;
    runtime->mode_changed = 0U;
}

/**
 * @brief 检测调试模式变化并生成一次位置目标。
 *
 * 位置模式每次调用只生成当前周期的目标，不直接操作电机。正弦模式将
 * 使能时的实际角作为中心，先计算本轮输出，再推进相位，因此第一个
 * 周期从当前位置连续开始。
 *
 * 模式变化时清除 enable 并复位运行状态；use_ramp 切换时保留使能，
 * 但以当前实际角重建波形和斜坡，避免切换瞬间产生跳变。
 */
void WheelDebug_Update(WheelDebug_t *debug,
                       WheelDebugRuntime_t *runtime,
                       float current_angle_deg,
                       float dt_s)
{
    WheelDebugMode_t mode;
    uint8_t use_ramp;
    float amplitude_deg;
    uint32_t period_ms;
    float desired_angle_deg;

    if ((debug == NULL) || (runtime == NULL))
    {
        return;
    }

    if (dt_s <= 0.0f)
    {
        dt_s = 0.001f;
    }

    mode = debug->mode;
    if (!WheelDebug_IsValidMode(mode))
    {
        mode = WHEEL_DEBUG_DISABLED;
        debug->mode = mode;
    }

    /* 模式变化是一次性事件，供 App 立即停止执行器。 */
    runtime->mode_changed = 0U;
    if (mode != runtime->previous_mode)
    {
        debug->enable = 0U;
        WheelDebug_ResetRuntime(runtime);
        runtime->previous_mode = mode;
        runtime->mode_changed = 1U;
    }

    /*
     * 速度和电压模式不需要位置生成；禁用时也不推进波形和斜坡状态。
     * 非位置模式的设备选择由 App 根据模式辅助函数完成。
     */
    if (!WheelDebug_IsPositionMode(mode) ||
        (debug->enable == 0U))
    {
        runtime->initialized = 0U;
        return;
    }

    /* 第一次使能或斜坡开关变化时，重新建立无扰动的目标起点。 */
    use_ramp = (debug->use_ramp != 0U) ? 1U : 0U;
    if ((runtime->initialized == 0U) ||
        (use_ramp != runtime->ramp_active))
    {
        runtime->sine_origin_deg =
            WheelDebug_Wrap360(current_angle_deg);
        runtime->sine_phase = 0.0f;
        runtime->ramp_active = use_ramp;
        runtime->initialized = 1U;
        SetpointRamp_Reset(&runtime->ramp,
                           runtime->sine_origin_deg);
        runtime->ramp_output_angle_deg =
            runtime->sine_origin_deg;
    }

    if (mode == WHEEL_DEBUG_GM_POSITION_SINE)
    {
        /* 幅值取绝对值并限制到单圈最大半幅，周期限制防止过高频跟踪。 */
        amplitude_deg = fabsf(debug->wave_amplitude_deg);
        amplitude_deg =
            WheelDebug_ClampAbs(amplitude_deg,
                                WHEEL_DEBUG_MAX_AMPLITUDE_DEG);
        period_ms = debug->wave_period_ms;
        if (period_ms < WHEEL_DEBUG_MIN_PERIOD_MS)
        {
            period_ms = WHEEL_DEBUG_MIN_PERIOD_MS;
        }

        desired_angle_deg =
            runtime->sine_origin_deg +
            (amplitude_deg *
             sinf(WHEEL_DEBUG_TWO_PI * runtime->sine_phase));

        runtime->sine_phase += dt_s / ((float)period_ms * 0.001f);
        if (runtime->sine_phase >= 1.0f)
        {
            runtime->sine_phase -= 1.0f;
        }
    }
    else
    {
        desired_angle_deg = debug->target_angle_deg;
    }

    /* 原始目标先归一化，保留给遥测观察，再进入 180 度等价选择。 */
    runtime->source_angle_deg =
        WheelDebug_Wrap360(desired_angle_deg);

    /*
     * direction_erpm 只用于 AngleOpt 选择物理等价方向，不会发送给 VESC。
     * 即使其为 0，AngleOpt 仍会选择距离当前角更近的朝向。
     */
    AngleOpt_Select(current_angle_deg,
                    runtime->source_angle_deg,
                    debug->direction_erpm,
                    &runtime->angle_result);

    /* 斜坡开关决定位置环看到原始优化目标还是平滑后的参考角。 */
    if (runtime->ramp_active != 0U)
    {
        SetpointRamp_SetDesired(
            &runtime->ramp,
            runtime->angle_result.target_angle_deg);
        SetpointRamp_Update(&runtime->ramp, dt_s);
        runtime->ramp_output_angle_deg =
            SetpointRamp_GetOutput(&runtime->ramp);
    }
    else
    {
        runtime->ramp_output_angle_deg =
            runtime->angle_result.target_angle_deg;
    }
}

/** @brief 判断位置模式；供 App 决定是否要求 GM6020 在线。 */
uint8_t WheelDebug_ModeUsesGmPosition(WheelDebugMode_t mode)
{
    return WheelDebug_IsPositionMode(mode);
}

/** @brief 判断 GM6020 速度模式。 */
uint8_t WheelDebug_ModeUsesGmSpeed(WheelDebugMode_t mode)
{
    return (mode == WHEEL_DEBUG_GM_SPEED) ? 1U : 0U;
}

/** @brief 判断 GM6020 直接电压模式。 */
uint8_t WheelDebug_ModeUsesGmVoltage(WheelDebugMode_t mode)
{
    return (mode == WHEEL_DEBUG_GM_VOLTAGE) ? 1U : 0U;
}

/** @brief 判断 VESC 调试模式。 */
uint8_t WheelDebug_ModeUsesVesc(WheelDebugMode_t mode)
{
    return (mode == WHEEL_DEBUG_VESC_SPEED) ? 1U : 0U;
}
