#ifndef ANGLE_OPT_HPP
#define ANGLE_OPT_HPP

#include <cstdint>

namespace steering::angle_opt
{

struct Result
{
    float target_angle_deg;
    int32_t target_erpm;
    uint8_t reversed;
};

Result select(float current_angle_deg,
              float target_angle_deg,
              int32_t target_erpm);

} // namespace steering::angle_opt

#endif
