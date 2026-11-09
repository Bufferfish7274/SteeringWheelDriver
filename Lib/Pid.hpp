#ifndef PID_HPP
#define PID_HPP

#include <cstdint>

/**
 * @file Pid.hpp
 * @brief 带积分分离和条件抗饱和的单通道位置式 PID。
 *
 * Pid::Config 只保存静态参数，Pid 保存运行状态并执行周期计算。配置在
 * Pid::init() 中一次性注入，之后每个控制周期调用 Pid::update()。
 */

namespace steering
{

/**
 * @brief 一个具有独立运行状态的位置式 PID 控制器。
 *
 * Pid 不得按值复制。对象创建和配置完成是两个阶段：
 * 1. 默认构造只建立对象存储；
 * 2. init() 写入参数并清除运行状态。
 *
 * 完成 init() 后，才能在控制周期中调用 update()。控制周期 dt 必须为
 * 正值；init() 不会用隐藏的默认值替代非法或未配置的周期。
 *
 * 为保持 Keil Watch 的现有调参路径，调参和诊断字段均为 public。正式
 * 运行时仍应把它们视为控制器内部状态，只允许本模块方法修改。
 */
class Pid
{
public:
    /**
     * @brief PID 的静态配置参数。
     *
     * 该结构体只描述“控制器应该怎样配置”，不包含积分、误差等运行状态，
     * 因此可以由 AppConfig 或其他调用者安全地按值复制和传递。
     *
     * 参数单位由调用者保持一致：
     * - kp 的单位为“输出/误差”；
     * - ki 的单位为“输出/(误差*秒)”；
     * - kd 的单位为“输出*秒/误差”。
     */
    struct Config
    {
        float kp;   // 比例增益
        float ki;   // 积分增益
        float kd;   // 微分增益

        /**
         * @brief 积分项的最大绝对值。
         * @note 限制的是已经乘过 ki 和 dt 的积分输出，不是原始误差累加值。
         */
        float integral_max;

        /** 非 0 时启用积分分离，即大误差阶段暂停本周期积分。 */
        uint8_t integral_separation;

        /** 积分分离阈值，单位与输入误差相同。 */
        float separation_threshold;
    };

    /**
     * @brief 建立一个尚未配置的 PID 对象。
     * @note 默认构造不写入产品参数；调用 update() 前必须先调用 init()。
     */
    Pid() = default;

    /* 禁止复制，避免两个对象带着相同的积分和误差状态独立运行。 */
    Pid(const Pid &) = delete;
    Pid &operator=(const Pid &) = delete;

    /**
     * @brief 注入配置并清除运行状态。
     * @param config PID 静态参数。
     * @param control_dt_s 控制周期，单位秒，必须为正值。
     * @param output_limit 输出绝对值上限，函数内部取绝对值。
     * @note integral_max 和 separation_threshold 也会被规范化为绝对值。
     */
    void init(const Config &config,
              float control_dt_s,
              float output_limit);

    /**
     * @brief 清除误差、积分、微分和输出，但保留整定参数。
     * @note 首次使能、故障恢复或控制器模式切换时必须调用。
     */
    void reset();

    /**
     * @brief 使用调用者已经处理好的误差执行一次 PID 计算。
     *
     * 角度控制应先计算最短角误差，再调用本函数：
     * @code
     * const float error = wrap180(target_angle - current_angle);
     * const float output = pid.update(error);
     * @endcode
     *
     * @param error 当前误差，单位由调用者定义。
     * @return 经过 output_max 限幅的控制输出。
     */
    float update(float error);

    /**
     * @brief 使用 target-feedback 作为误差执行一次 PID 计算。
     *
     * 本入口适合 RPM、电流等线性量。周期性角度不能直接使用本入口，
     * 因为 359° 与 1° 的普通差值不是最短角误差。
     *
     * @param target 当前目标值。
     * @param feedback 当前反馈值。
     * @return 经过 output_max 限幅的控制输出。
     */
    float update(float target, float feedback);

    /*
     * 整定参数。为保留 Keil Watch 的 position_pid.kp 等调参路径而设为
     * public；正式运行时应通过重新烧录 AppConfig 固化参数，不应在控制
     * 周期中随意修改。
     */
    float kp;
    float ki;
    float kd;

    /** 控制周期，单位秒；由 init() 写入，必须保持正值。 */
    float dt;

    /** 输出绝对值上限，同时用于条件抗积分饱和判断。 */
    float output_max;

    /** 积分项绝对值上限；限制的是积分输出，而非原始误差和。 */
    float integral_max;

    /** 非 0 时启用积分分离。 */
    uint8_t integral_separation;

    /** 启用积分分离时，误差超过该绝对值即暂停本周期积分。 */
    float separation_threshold;

    /*
     * 运行状态和诊断字段。两参数 update(target, feedback) 会更新 target
     * 和 feedback；直接使用 update(error) 时，这两个字段不参与计算，可能
     * 保留上一次调用值。
     */
    float target;
    float feedback;

    /** 本次计算的输入误差。 */
    float error;

    /** 上一次输入误差，用于计算微分项。 */
    float last_error;

    /** 本次经过限幅后的控制输出。 */
    float output;

    /** 本次比例项贡献。 */
    float proportional;

    /**
     * 当前积分项贡献。
     * @note 已乘过 ki 和 dt，是积分输出而不是原始误差累加值。
     */
    float integral;

    /** 本次微分项贡献。 */
    float differential;
};

} // namespace steering

#endif
