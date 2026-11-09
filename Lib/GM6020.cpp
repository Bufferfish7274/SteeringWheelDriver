#include "GM6020.hpp"

#include <cmath>
#include <cstring>

namespace steering
{

namespace
{

bool isValidConfig(const Gm6020::Config &config)
{
    return ((config.id >= 1U) && (config.id <= 7U) &&
            (config.zero_raw <= 0x1FFFU) &&
            ((config.angle_sign == 1) ||
             (config.angle_sign == -1)) &&
            ((config.voltage_sign == 1) ||
             (config.voltage_sign == -1)) &&
            std::isfinite(config.gear_ratio) &&
            (config.gear_ratio > 0.0f) &&
            std::isfinite(config.control_dt_s) &&
            (config.control_dt_s > 0.0f) &&
            std::isfinite(config.max_speed_rpm) &&
            (config.max_speed_rpm > 0.0f) &&
            (config.voltage_limit_raw > 0) &&
            (config.voltage_limit_raw <=
             Gm6020::MaxVoltageRaw) &&
            (config.online_timeout_ms > 0U));
}

} // namespace

void Gm6020::init(CAN_HandleTypeDef *handle,
                  const Config &config)
{
    if ((handle == nullptr) || !isValidConfig(config))
    {
        Error_Handler();
        return;
    }

    hcan = handle;
    id = static_cast<uint8_t>(config.id);
    zero_raw = static_cast<uint16_t>(config.zero_raw);
    angle_sign = static_cast<int8_t>(config.angle_sign);
    voltage_sign = static_cast<int8_t>(config.voltage_sign);
    gear_ratio = config.gear_ratio;

    raw_angle = 0U;
    angle_deg = 0.0f;
    speed_rpm = 0;
    current_raw = 0;
    temperature = 0U;
    is_online = 0U;
    is_enabled = 0U;
    last_rx_ms = 0U;

    control_mode = ControlMode::Position;
    target_angle_deg = 0.0f;
    requested_speed_rpm = 0.0f;
    target_speed_rpm = 0.0f;
    requested_voltage_raw = 0;
    target_voltage = 0;

    last_tx_status = HAL_ERROR;
    tx_ok_count = 0U;
    tx_busy_count = 0U;
    tx_error_count = 0U;
    last_tx_std_id = 0U;
    last_tx_voltage = 0;
    std::memset(last_tx_data, 0, sizeof(last_tx_data));

    max_speed_rpm = config.max_speed_rpm;
    voltage_limit_raw =
        static_cast<int16_t>(config.voltage_limit_raw);
    online_timeout_ms = config.online_timeout_ms;

    position_pid.init(
        config.position,
        config.control_dt_s,
        max_speed_rpm);
    speed_pid.init(
        config.speed,
        config.control_dt_s,
        static_cast<float>(voltage_limit_raw));
}

float Gm6020::wrap360(float angle_deg)
{
    angle_deg = std::fmod(angle_deg, DegPerRev);
    if (angle_deg < 0.0f)
    {
        angle_deg += DegPerRev;
    }
    return angle_deg;
}

float Gm6020::wrap180(float angle_deg)
{
    angle_deg = wrap360(angle_deg);
    if (angle_deg >= 180.0f)
    {
        angle_deg -= DegPerRev;
    }
    return angle_deg;
}

float Gm6020::clampAbs(float value, float limit)
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

float Gm6020::rawToAngle(uint16_t raw) const
{
    const int32_t delta =
        static_cast<int32_t>(raw) - static_cast<int32_t>(zero_raw);
    float motor_deg =
        static_cast<float>(delta) *
        (DegPerRev / RawPerRev) *
        static_cast<float>(angle_sign);

    if ((gear_ratio > 0.0f) && (gear_ratio != 1.0f))
    {
        motor_deg /= gear_ratio;
    }
    return wrap360(motor_deg);
}

int16_t Gm6020::clampVoltage(float voltage_raw) const
{
    float limit = std::fabs(static_cast<float>(voltage_limit_raw));
    if (limit > static_cast<float>(MaxVoltageRaw))
    {
        limit = static_cast<float>(MaxVoltageRaw);
    }
    if (limit < 1.0f)
    {
        return 0;
    }
    return static_cast<int16_t>(clampAbs(voltage_raw, limit));
}

void Gm6020::updateControlLimits()
{
    position_pid.output_max = std::fabs(max_speed_rpm);
    speed_pid.output_max =
        std::fabs(static_cast<float>(voltage_limit_raw));
}

void Gm6020::onRx(const CAN_RxHeaderTypeDef *header,
                  const uint8_t data[8],
                  uint32_t now_ms)
{
    if ((header == nullptr) || (data == nullptr))
    {
        return;
    }
    if ((header->IDE != CAN_ID_STD) ||
        (header->RTR != CAN_RTR_DATA) ||
        (header->DLC != 8U) ||
        (header->StdId !=
         (static_cast<uint32_t>(0x204U) + id)))
    {
        return;
    }

    const uint16_t raw =
        static_cast<uint16_t>(
            (static_cast<uint16_t>(data[0]) << 8U) |
            static_cast<uint16_t>(data[1]));
    const int16_t speed =
        static_cast<int16_t>(
            (static_cast<uint16_t>(data[2]) << 8U) |
            static_cast<uint16_t>(data[3]));
    const bool was_online = (is_online != 0U);

    raw_angle = raw;
    angle_deg = rawToAngle(raw);
    speed_rpm = static_cast<int16_t>(
        static_cast<int32_t>(angle_sign) *
        static_cast<int32_t>(speed));
    current_raw = static_cast<int16_t>(
        (static_cast<uint16_t>(data[4]) << 8U) |
        static_cast<uint16_t>(data[5]));
    temperature = data[6];
    last_rx_ms = now_ms;

    if (!was_online)
    {
        target_angle_deg = angle_deg;
        requested_speed_rpm = 0.0f;
        target_speed_rpm = 0.0f;
        requested_voltage_raw = 0;
        target_voltage = 0;
        position_pid.reset();
        speed_pid.reset();
    }
    is_online = 1U;
}

void Gm6020::setTargetAngle(float angle)
{
    target_angle_deg = wrap360(angle);
}

void Gm6020::setTargetSpeed(float speed)
{
    requested_speed_rpm = speed;
}

void Gm6020::setTargetVoltage(int16_t voltage)
{
    if (voltage > MaxVoltageRaw)
    {
        voltage = static_cast<int16_t>(MaxVoltageRaw);
    }
    else if (voltage < -MaxVoltageRaw)
    {
        voltage = static_cast<int16_t>(-MaxVoltageRaw);
    }
    requested_voltage_raw = voltage;
}

void Gm6020::setControlMode(ControlMode mode)
{
    if (mode == control_mode)
    {
        return;
    }
    if ((mode != ControlMode::Position) &&
        (mode != ControlMode::Speed) &&
        (mode != ControlMode::Voltage))
    {
        return;
    }

    disable();
    requested_speed_rpm = 0.0f;
    requested_voltage_raw = 0;
    target_speed_rpm = 0.0f;
    target_voltage = 0;
    position_pid.reset();
    speed_pid.reset();
    control_mode = mode;
    sendVoltage(0);
}

void Gm6020::enable()
{
    if (is_online == 0U)
    {
        is_enabled = 0U;
        return;
    }

    target_angle_deg = angle_deg;
    requested_speed_rpm = 0.0f;
    target_speed_rpm = 0.0f;
    requested_voltage_raw = 0;
    target_voltage = 0;
    position_pid.reset();
    speed_pid.reset();
    is_enabled = 1U;
}

void Gm6020::disable()
{
    is_enabled = 0U;
    requested_speed_rpm = 0.0f;
    target_speed_rpm = 0.0f;
    requested_voltage_raw = 0;
    target_voltage = 0;
    if (is_online != 0U)
    {
        target_angle_deg = angle_deg;
    }
    position_pid.reset();
    speed_pid.reset();
    sendVoltage(0);
}

void Gm6020::controlStep(uint32_t now_ms)
{
    if ((is_online != 0U) &&
        ((now_ms - last_rx_ms) > online_timeout_ms))
    {
        is_online = 0U;
        is_enabled = 0U;
        requested_speed_rpm = 0.0f;
        target_speed_rpm = 0.0f;
        requested_voltage_raw = 0;
        target_voltage = 0;
        position_pid.reset();
        speed_pid.reset();
        sendVoltage(0);
        return;
    }

    if ((is_enabled == 0U) || (is_online == 0U))
    {
        target_speed_rpm = 0.0f;
        target_voltage = 0;
        sendVoltage(0);
        return;
    }

    updateControlLimits();

    float target_rpm = 0.0f;
    float voltage_cmd = 0.0f;

    switch (control_mode)
    {
        case ControlMode::Position:
        {
            const float error =
                wrap180(target_angle_deg - angle_deg);
            target_rpm = position_pid.update(error);
            target_rpm = clampAbs(target_rpm, max_speed_rpm);
            voltage_cmd = speed_pid.update(
                target_rpm,
                static_cast<float>(speed_rpm));
            voltage_cmd *= static_cast<float>(voltage_sign);
            break;
        }
        case ControlMode::Speed:
        {
            target_rpm =
                clampAbs(requested_speed_rpm, max_speed_rpm);
            voltage_cmd = speed_pid.update(
                target_rpm,
                static_cast<float>(speed_rpm));
            voltage_cmd *= static_cast<float>(voltage_sign);
            break;
        }
        case ControlMode::Voltage:
            voltage_cmd =
                static_cast<float>(requested_voltage_raw);
            break;
        default:
            break;
    }

    target_speed_rpm = target_rpm;
    target_voltage = clampVoltage(voltage_cmd);
    sendVoltage(target_voltage);
}

HAL_StatusTypeDef Gm6020::sendVoltage(int16_t voltage_raw)
{
    if ((hcan == nullptr) || (id < 1U) || (id > 7U))
    {
        last_tx_status = HAL_ERROR;
        ++tx_error_count;
        return HAL_ERROR;
    }

    uint8_t data[8] = {};
    uint32_t frame_id;
    uint8_t slot;

    int32_t limited_voltage = voltage_raw;
    if (limited_voltage > MaxVoltageRaw)
    {
        limited_voltage = MaxVoltageRaw;
    }
    else if (limited_voltage < -MaxVoltageRaw)
    {
        limited_voltage = -MaxVoltageRaw;
    }

    if (id <= 4U)
    {
        frame_id = 0x1FFU;
        slot = static_cast<uint8_t>(id - 1U);
    }
    else
    {
        frame_id = 0x2FFU;
        slot = static_cast<uint8_t>(id - 5U);
    }

    const uint16_t encoded =
        static_cast<uint16_t>(limited_voltage);
    data[slot * 2U] = static_cast<uint8_t>(encoded >> 8U);
    data[slot * 2U + 1U] = static_cast<uint8_t>(encoded);

    CAN_TxHeaderTypeDef header = {};
    header.StdId = frame_id;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;

    last_tx_std_id = frame_id;
    last_tx_voltage =
        static_cast<int16_t>(limited_voltage);

    uint32_t tx_mailbox = 0U;
    last_tx_status =
        HAL_CAN_AddTxMessage(hcan, &header, data, &tx_mailbox);
    std::memcpy(last_tx_data, data, sizeof(last_tx_data));

    if (last_tx_status == HAL_OK)
    {
        ++tx_ok_count;
    }
    else if (last_tx_status == HAL_BUSY)
    {
        ++tx_busy_count;
    }
    else
    {
        ++tx_error_count;
    }
    return last_tx_status;
}

} // namespace steering
