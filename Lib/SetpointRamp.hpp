#ifndef SETPOINT_RAMP_HPP
#define SETPOINT_RAMP_HPP

namespace steering
{

struct SetpointRampConfig
{
    float max_speed_deg_s;
    float max_accel_deg_s2;
};

class SetpointRamp
{
public:
    SetpointRamp() = default;
    SetpointRamp(const SetpointRamp &) = delete;
    SetpointRamp &operator=(const SetpointRamp &) = delete;

    void init(float initial_angle_deg,
              const SetpointRampConfig &config);
    void reset(float current_angle_deg);
    void setDesired(float desired_angle_deg);
    void update(float dt_s);

    float output() const;
    float velocity() const;

    float desired_angle_deg;
    float output_angle_deg;
    float velocity_deg_s;
    float max_speed_deg_s;
    float max_accel_deg_s2;

private:
    static float wrap360(float angle_deg);
    static float wrap180(float angle_deg);
    static float clamp(float value, float limit);
};

} // namespace steering

#endif
