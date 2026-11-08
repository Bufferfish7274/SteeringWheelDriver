#ifndef GM6020_HPP
#define GM6020_HPP

#include <cstdint>

#include "Pid.hpp"
#include "main.h"

namespace steering
{

class Gm6020
{
public:
    enum class ControlMode : uint8_t
    {
        Position = 0U,
        Speed,
        Voltage
    };

    struct Config
    {
        uint32_t id;
        uint32_t zero_raw;
        int32_t angle_sign;
        int32_t voltage_sign;
        float gear_ratio;

        PidConfig position;
        PidConfig speed;
        float control_dt_s;
        float max_speed_rpm;
        int32_t voltage_limit_raw;
        uint32_t online_timeout_ms;
    };

    Gm6020() = default;
    Gm6020(const Gm6020 &) = delete;
    Gm6020 &operator=(const Gm6020 &) = delete;

    void init(CAN_HandleTypeDef *hcan, const Config &config);
    void onRx(const CAN_RxHeaderTypeDef *header,
              const uint8_t data[8],
              uint32_t now_ms);
    void setTargetAngle(float angle_deg);
    void setTargetSpeed(float speed_rpm);
    void setTargetVoltage(int16_t voltage_raw);
    void setControlMode(ControlMode mode);
    void enable();
    void disable();
    void controlStep(uint32_t now_ms);
    HAL_StatusTypeDef sendVoltage(int16_t voltage_raw);

    static float wrap180(float angle_deg);

    static constexpr float kRawPerRev = 8192.0f;
    static constexpr float kDegPerRev = 360.0f;
    static constexpr int32_t kMaxVoltageRaw = 25000;

    CAN_HandleTypeDef *hcan;
    uint8_t id;
    uint16_t zero_raw;
    int8_t angle_sign;
    int8_t voltage_sign;
    float gear_ratio;

    uint16_t raw_angle;
    float angle_deg;
    int16_t speed_rpm;
    int16_t current_raw;
    uint8_t temperature;
    uint8_t is_online;
    uint8_t is_enabled;
    uint32_t last_rx_ms;

    ControlMode control_mode;
    float target_angle_deg;
    float requested_speed_rpm;
    float target_speed_rpm;
    int16_t requested_voltage_raw;
    int16_t target_voltage;

    HAL_StatusTypeDef last_tx_status;
    uint32_t tx_ok_count;
    uint32_t tx_busy_count;
    uint32_t tx_error_count;
    uint32_t last_tx_std_id;
    int16_t last_tx_voltage;
    uint8_t last_tx_data[8];

    float max_speed_rpm;
    int16_t voltage_limit_raw;
    uint32_t online_timeout_ms;

    Pid position_pid;
    Pid speed_pid;

private:
    static float wrap360(float angle_deg);
    static float clampAbs(float value, float limit);
    float rawToAngle(uint16_t raw_angle) const;
    int16_t clampVoltage(float voltage_raw) const;
    void updateControlLimits();
};

} // namespace steering

#endif
