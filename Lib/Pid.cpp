#include "Pid.hpp"

#include <cmath>

namespace steering
{

namespace
{

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

} // namespace

void Pid::init(const PidConfig &config,
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

float Pid::update(float current_error)
{
    float integral_next = integral;
    bool integrate = true;

    proportional = kp * current_error;
    differential = 0.0f;
    if (kd != 0.0f)
    {
        differential =
            kd * (current_error - last_error) / dt;
    }

    if ((integral_separation != 0U) &&
        (std::fabs(current_error) > separation_threshold))
    {
        integrate = false;
    }

    if (integrate)
    {
        integral_next += ki * current_error * dt;
        integral_next = clampAbs(integral_next, integral_max);
    }

    output = proportional + integral_next + differential;

    if (((output > output_max) && (current_error > 0.0f)) ||
        ((output < -output_max) && (current_error < 0.0f)))
    {
        integral_next = integral;
        output = proportional + integral_next + differential;
    }

    integral = integral_next;
    error = current_error;
    last_error = current_error;
    output = clampAbs(output, output_max);
    return output;
}

float Pid::update(float current_target, float current_feedback)
{
    target = current_target;
    feedback = current_feedback;
    return update(current_target - current_feedback);
}

} // namespace steering
