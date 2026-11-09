#include "AngleOpt.hpp"

#include <cmath>

namespace steering::angle_opt
{

namespace
{

float wrap360(float angle_deg)
{
    angle_deg = std::fmod(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float wrap180(float angle_deg)
{
    angle_deg = std::fmod(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    if (angle_deg >= 180.0f)
    {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

struct Candidate
{
    float angle_deg;
    int32_t erpm;
    uint8_t reversed;
};

} // namespace

Result select(float current_angle_deg,
              float target_angle_deg,
              int32_t target_erpm)
{
    const float current_norm = wrap360(current_angle_deg);
    const float target_norm = wrap360(target_angle_deg);
    const Candidate normal =
    {
        target_norm,
        target_erpm,
        0U
    };
    const Candidate reversed =
    {
        wrap360(target_norm + 180.0f),
        -target_erpm,
        1U
    };

    const float normal_error =
        std::fabs(wrap180(normal.angle_deg - current_norm));
    const float reversed_error =
        std::fabs(wrap180(reversed.angle_deg - current_norm));
    const Candidate selected =
        (reversed_error < normal_error) ? reversed : normal;

    Result result;
    result.target_angle_deg = selected.angle_deg;
    result.target_erpm = selected.erpm;
    result.reversed = selected.reversed;
    return result;
}

} // namespace steering::angle_opt
