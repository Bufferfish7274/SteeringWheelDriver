#ifndef WHEEL_DEBUG_H
#define WHEEL_DEBUG_H

/**
 * @file WheelDebug.h
 * @brief 调试目标输入和 Watch 可调参数。
 *
 * 本模块只生成“希望电机跟随什么”，不直接使能电机或发送 CAN 帧。App
 * 在 1 kHz 控制步骤中调用 WheelDebug_Update()，再把生成的位置、速度、
 * 电压或 ERPM 交给对应驱动。因此位置阶跃、正弦与正式位置控制可以共享
 * AngleOpt、SetpointRamp 和 GM6020 位置环。
 */

#include <stdint.h>

#include "angle_opt.h"
#include "setpoint_ramp.h"

/** @brief 调试固件在 Watch 中选择的控制目标类型。 */
typedef enum
{
    /* 两个执行器都保持失能并持续输出零；也用于观察原始角和零位。 */
    WHEEL_DEBUG_DISABLED = 0,
    /* 使用 target_angle_deg 做 GM6020 位置阶跃。 */
    WHEEL_DEBUG_GM_POSITION_STEP,
    /* 在使能时的实际角附近生成正弦位置目标。 */
    WHEEL_DEBUG_GM_POSITION_SINE,
    /* 直接把目标 RPM 送入 GM6020 速度环。 */
    WHEEL_DEBUG_GM_SPEED,
    /* 经过电压限幅后直接发送 GM6020 电压 raw。 */
    WHEEL_DEBUG_GM_VOLTAGE,
    /* 只测试 VESC CAN 控制，GM6020 保持失能。 */
    WHEEL_DEBUG_VESC_SPEED
} WheelDebugMode_t;

/** @brief 调试固件的 Watch 默认值和台架安全边界。 */
typedef struct
{
    float target_angle_deg;
    int32_t direction_erpm;
    float target_speed_rpm;
    int32_t target_voltage_raw;
    int32_t target_erpm;
    int32_t erpm_limit;
    uint8_t use_ramp;
    float wave_amplitude_deg;
    uint32_t wave_period_ms;
    uint32_t min_period_ms;
    float max_amplitude_deg;
} WheelDebug_Config_t;

/**
 * @brief Keil Watch 直接编辑的调试输入。
 *
 * volatile 只保证每次访问重新读取内存；多字段之间没有事务语义。因此
 * 切换模式时应先把 enable 清零，再修改目标，最后重新使能。mode 变化
 * 会由 WheelDebug_Update() 自动清除 enable，避免切换后沿用旧输入。
 */
typedef struct
{
    /* 当前调试子模式；非法值会回退到 DISABLED。 */
    volatile WheelDebugMode_t mode;
    /* 调试总使能；任何模式转换和反馈超时都会把它清零。 */
    volatile uint8_t enable;

    /* GM6020 位置阶跃目标，单位度。 */
    volatile float target_angle_deg;
    /* 位置模式中仅用于 AngleOpt，不向 VESC 发送。 */
    volatile int32_t direction_erpm;

    /* GM6020 速度模式目标，单位机械 RPM。 */
    volatile float target_speed_rpm;
    /* GM6020 直接电压目标，单位为控制器 raw 值。 */
    volatile int16_t target_voltage_raw;

    /* VESC 速度目标，单位电气 RPM。 */
    volatile int32_t target_erpm;
    /* VESC 调试命令绝对值上限。 */
    volatile int32_t erpm_limit;

    /* 1 表示位置目标经过 SetpointRamp，0 表示直接进入位置环。 */
    volatile uint8_t use_ramp;
    /* 正弦幅值，实际使用时限定在 0..180 度。 */
    volatile float wave_amplitude_deg;
    /* 正弦完整周期，实际使用时最小为 200 ms。 */
    volatile uint32_t wave_period_ms;
} WheelDebug_t;

/**
 * @brief 不由 Watch 编辑的调试运行状态。
 *
 * previous_mode 用于检测模式切换；正弦原点、相位和斜坡状态用于保证
 * 使能及参数切换时目标连续。App 会把该状态放入 App_t，便于 Watch
 * 展开观察中间结果。
 */
typedef struct
{
    /* 最近一次 AngleOpt 的最终目标和方向标志。 */
    AngleOptResult_t angle_result;
    /* 上一次已经处理过的模式，用来生成一次 mode_changed 事件。 */
    WheelDebugMode_t previous_mode;
    /* 角度优化前的目标，保留 0..360。 */
    float source_angle_deg;
    /* 经过可选斜坡后真正交给 GM6020 的位置目标。 */
    float ramp_output_angle_deg;
    /* 正弦中心，使能或斜坡切换时取当前实际角。 */
    float sine_origin_deg;
    /* 当前波形归一化相位，范围 [0, 1)。 */
    float sine_phase;
    /* 使能后是否已经按当前位置完成初始化。 */
    uint8_t initialized;
    /* 本次运行所采用的斜坡开关值。 */
    uint8_t ramp_active;
    /* 仅在一次控制步骤内为 1 的模式变化事件。 */
    uint8_t mode_changed;
    /* 从 AppConfig 注入的调试安全边界。 */
    uint32_t min_period_ms;
    float max_amplitude_deg;
} WheelDebugRuntime_t;

/** @brief 全局调试输入，必须在 Watch 中直接修改。 */
extern WheelDebug_t g_wheel_debug;

/**
 * @brief 使用应用层配置恢复 Watch 默认值并清空调试运行状态。
 * @param debug 调试输入对象。
 * @param runtime 调试运行状态；previous_mode 会同步为 DISABLED。
 * @param config 调试默认值和台架安全边界。
 */
void WheelDebug_Init(WheelDebug_t *debug,
                     WheelDebugRuntime_t *runtime,
                     const WheelDebug_Config_t *config);

/**
 * @brief 清空波形相位和 AngleOpt 结果。
 * @note 不修改 g_wheel_debug 中的目标及使能输入；使能状态由 App 管理。
 */
void WheelDebug_ResetRuntime(WheelDebugRuntime_t *runtime);

/**
 * @brief 更新调试模式状态并生成位置模式的目标角。
 *
 * 非位置模式也会执行模式变化检测，但不会修改 angle_result 和斜坡输出。
 * mode_changed 只在检测到模式变化后的本次调用中置 1。
 *
 * @param debug Watch 调试输入。
 * @param runtime 上一次调用的运行状态及本次输出。
 * @param ramp 正式与调试路径共用的位置参考斜坡。
 * @param current_angle_deg GM6020 当前舵轮角，供正弦起相和 AngleOpt 使用。
 * @param dt_s 本次控制周期，秒；必须为正值。
 */
void WheelDebug_Update(WheelDebug_t *debug,
                       WheelDebugRuntime_t *runtime,
                       SetpointRamp_t *ramp,
                       float current_angle_deg,
                       float dt_s);

/** @brief 返回模式是否使用 GM6020 位置环。 */
uint8_t WheelDebug_ModeUsesGmPosition(WheelDebugMode_t mode);

/** @brief 返回模式是否使用 GM6020 速度环。 */
uint8_t WheelDebug_ModeUsesGmSpeed(WheelDebugMode_t mode);

/** @brief 返回模式是否使用 GM6020 直接电压控制。 */
uint8_t WheelDebug_ModeUsesGmVoltage(WheelDebugMode_t mode);

/** @brief 返回模式是否使用 VESC。 */
uint8_t WheelDebug_ModeUsesVesc(WheelDebugMode_t mode);

#endif /* WHEEL_DEBUG_H */
