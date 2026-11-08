#ifndef PID_HPP
#define PID_HPP

#include <cstdint>

namespace steering
{

struct PidConfig
{
    float kp;
    float ki;
    float kd;
    float integral_max;
    uint8_t integral_separation;
    float separation_threshold;
};

class Pid
{
public:
    Pid() = default;
    Pid(const Pid &) = delete;
    Pid &operator=(const Pid &) = delete;

    void init(const PidConfig &config,
              float control_dt_s,
              float output_limit);
    void reset();

    float update(float error);
    float update(float target, float feedback);

    float kp;
    float ki;
    float kd;
    float dt;
    float output_max;
    float integral_max;
    uint8_t integral_separation;
    float separation_threshold;

    float target;
    float feedback;
    float error;
    float last_error;
    float output;
    float proportional;
    float integral;
    float differential;
};

} // namespace steering

#endif
