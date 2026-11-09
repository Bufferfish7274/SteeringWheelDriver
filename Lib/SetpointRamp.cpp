#include "SetpointRamp.hpp"

#include <cmath>

namespace steering
{

namespace
{

constexpr float AngleEpsDeg = 0.05f;
constexpr float VelocityEpsDegS = 0.5f;
constexpr float DegPerRev = 360.0f;

} // namespace

float SetpointRamp::wrap360(float angle_deg)
{
    angle_deg = std::fmod(angle_deg, DegPerRev);
    if (angle_deg < 0.0f)
    {
        angle_deg += DegPerRev;
    }
    return angle_deg;
}

float SetpointRamp::wrap180(float angle_deg)
{
    angle_deg = wrap360(angle_deg);
    if (angle_deg >= 180.0f)
    {
        angle_deg -= DegPerRev;
    }
    return angle_deg;
}

float SetpointRamp::clamp(float value, float limit)
{
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

void SetpointRamp::init(float initial_angle_deg,
                        const SetpointRampConfig &config)
{
    max_speed_deg_s = std::fabs(config.max_speed_deg_s);
    max_accel_deg_s2 = std::fabs(config.max_accel_deg_s2);

    desired_angle_deg = wrap360(initial_angle_deg);
    output_angle_deg = desired_angle_deg;
    velocity_deg_s = 0.0f;
}

void SetpointRamp::reset(float current_angle_deg)
{
    output_angle_deg = wrap360(current_angle_deg);
    desired_angle_deg = output_angle_deg;
    velocity_deg_s = 0.0f;
}

void SetpointRamp::setDesired(float desired)
{
    desired_angle_deg = wrap360(desired);
}

void SetpointRamp::update(float dt_s)
{
    if (dt_s <= 0.0f)
    {
        return;
    }

    const float error_deg =
        wrap180(desired_angle_deg - output_angle_deg);

    if ((std::fabs(error_deg) <= AngleEpsDeg) &&
        (std::fabs(velocity_deg_s) <= VelocityEpsDegS))
    {
        output_angle_deg = desired_angle_deg;
        velocity_deg_s = 0.0f;
        return;
    }

    float speed_magnitude = std::sqrt(
        2.0f * max_accel_deg_s2 * std::fabs(error_deg));
    if (speed_magnitude > max_speed_deg_s)
    {
        speed_magnitude = max_speed_deg_s;
    }

    const float target_speed =
        (error_deg > 0.0f) ? speed_magnitude : -speed_magnitude;
    const float max_delta_speed = max_accel_deg_s2 * dt_s;
    const float speed_delta =
        clamp(target_speed - velocity_deg_s, max_delta_speed);

    velocity_deg_s += speed_delta;
    output_angle_deg = wrap360(
        output_angle_deg + velocity_deg_s * dt_s);
}

float SetpointRamp::output() const
{
    return output_angle_deg;
}

float SetpointRamp::velocity() const
{
    return velocity_deg_s;
}

} // namespace steering
