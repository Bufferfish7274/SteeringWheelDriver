#include "Pid.hpp"

#include <cmath>

/**
 * @file Pid.cpp
 * @brief 单通道位置式 PID 的参数注入、状态复位和周期计算。
 */

namespace steering
{

namespace
{

/**
 * @brief 将值限制在 [-limit, +limit]，仅供当前翻译单元使用。
 * @param value 待限制的值。
 * @param limit 绝对值上限；非正值表示允许范围为 0。
 * @return 限幅后的值。
 */
float clampAbs(float value, float limit)
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

} // namespace steering::{anonymous}

/**
 * @brief 写入静态参数并清除上一次运行的动态状态。
 *
 * output_limit、integral_max 和 separation_threshold 在这里统一转换为
 * 绝对值。dt 不做回退处理，因此调用者必须保证 control_dt_s > 0。
 */
void Pid::init(const Config &config,
               float control_dt_s,
               float output_limit)
{
    kp = config.kp;
    ki = config.ki;
    kd = config.kd;
    dt = control_dt_s;
    output_max = std::fabs(output_limit);
    integral_max = std::fabs(config.integral_max);
    integral_separation = config.integral_separation;
    separation_threshold = std::fabs(config.separation_threshold);
    reset();
}

/**
 * @brief 清除与历史控制过程相关的运行状态。
 *
 * 整定参数不会改变。该函数适合首次使能、故障恢复、模式切换以及重新
 * 建立控制目标之前调用。
 */
void Pid::reset()
{
    target = 0.0f;
    feedback = 0.0f;
    error = 0.0f;
    last_error = 0.0f;
    output = 0.0f;
    proportional = 0.0f;
    integral = 0.0f;
    differential = 0.0f;
}

/**
 * @brief 执行一次位置式 PID 计算。
 *
 * 计算顺序：
 * 1. 计算 P 和基于误差差分、控制周期的 D；
 * 2. 根据积分分离策略决定本周期是否累加 I；
 * 3. 用未限幅 P+I+D 判断条件抗积分饱和；
 * 4. 必要时回滚本周期积分增量；
 * 5. 保存运行状态并返回限幅输出。
 *
 * @note 调用前必须通过 init() 写入正数 dt。
 */
float Pid::update(float current_error)
{
    /* 候选积分先在局部变量中计算，确认未导致饱和后再提交。 */
    float integral_next = integral;
    bool integrate = true;

    proportional = kp * current_error;
    differential = 0.0f;

    /* kd 为 0 时跳过微分，避免无意义的除法和计算。 */
    if (kd != 0.0f)
    {
        differential =
            kd * (current_error - last_error) / dt;
    }

    /*
     * 积分分离只暂停本周期累加，不清空已有积分；误差回到阈值以内后
     * 积分可以从之前的状态继续。
     */
    if ((integral_separation != 0U) &&
        (std::fabs(current_error) > separation_threshold))
    {
        integrate = false;
    }

    /* 积分输出本身也做绝对限幅，防止长时间累积超出控制范围。 */
    if (integrate)
    {
        integral_next += ki * current_error * dt;
        integral_next = clampAbs(integral_next, integral_max);
    }

    output = proportional + integral_next + differential;

    /*
     * 条件抗积分饱和：
     * 若输出已向一侧饱和，而误差仍继续把输出推向同一方向，则回滚本次
     * 积分增量。误差反向时不回滚，使控制器可以更快退出饱和。
     */
    if (((output > output_max) && (current_error > 0.0f)) ||
        ((output < -output_max) && (current_error < 0.0f)))
    {
        integral_next = integral;
        output = proportional + integral_next + differential;
    }

    /* 提交本周期状态；last_error 必须在返回前更新，供下一周期微分使用。 */
    integral = integral_next;
    error = current_error;
    last_error = current_error;
    output = clampAbs(output, output_max);
    return output;
}

/**
 * @brief 为线性量计算 target-feedback 并执行一次 PID。
 *
 * 两参数入口同时保留 target 和 feedback 供遥测/Watch 观察。角度等周期性
 * 量应先由调用者解绕为最短误差，再使用一参数 update(error)。
 */
float Pid::update(float current_target, float current_feedback)
{
    target = current_target;
    feedback = current_feedback;
    return update(current_target - current_feedback);
}

} // namespace steering
