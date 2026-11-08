#include "MasterCan.hpp"

namespace steering
{

void MasterCan::init(const Config &config)
{
    if ((config.wheel_id < 1U) ||
        (config.wheel_id > 4U) ||
        (config.link_timeout_ms == 0U))
    {
        Error_Handler();
        return;
    }

    wheel_id = static_cast<uint8_t>(config.wheel_id);
    control_id = kControlIdBase + wheel_id;
    heartbeat_id = kHeartbeatIdBase + wheel_id;
    drive_erpm = 0;
    angle_code = 0U;
    target_angle_deg = 0.0f;
    last_control_ms = 0U;
    last_heartbeat_ms = 0U;
    link_timeout_ms = config.link_timeout_ms;
    control_seen = 0U;
    heartbeat_seen = 0U;
    link_valid = 0U;
}

void MasterCan::onRx(const CAN_RxHeaderTypeDef *header,
                     const uint8_t data[8],
                     uint32_t now_ms)
{
    if ((header == nullptr) || (data == nullptr))
    {
        return;
    }
    if ((header->IDE != CAN_ID_STD) ||
        (header->RTR != CAN_RTR_DATA) ||
        (header->DLC != 8U))
    {
        return;
    }

    if (header->StdId == control_id)
    {
        const uint32_t erpm_raw =
            (static_cast<uint32_t>(data[0]) << 24U) |
            (static_cast<uint32_t>(data[1]) << 16U) |
            (static_cast<uint32_t>(data[2]) << 8U) |
            static_cast<uint32_t>(data[3]);
        int32_t angle_raw = static_cast<int32_t>(
            (static_cast<uint32_t>(data[4]) << 24U) |
            (static_cast<uint32_t>(data[5]) << 16U) |
            (static_cast<uint32_t>(data[6]) << 8U) |
            static_cast<uint32_t>(data[7]));

        if (angle_raw < 0)
        {
            angle_raw = 0;
        }
        else if (angle_raw > 16383)
        {
            angle_raw = 16383;
        }

        drive_erpm = static_cast<int32_t>(erpm_raw);
        angle_code = static_cast<uint16_t>(angle_raw);
        target_angle_deg =
            static_cast<float>(angle_code) *
            (360.0f / 16384.0f);
        last_control_ms = now_ms;
        last_heartbeat_ms = now_ms;
        control_seen = 1U;
        heartbeat_seen = 1U;
        update(now_ms);
    }
    else if (header->StdId == heartbeat_id)
    {
        update(now_ms);
        last_heartbeat_ms = now_ms;
        heartbeat_seen = 1U;
        update(now_ms);
    }
}

void MasterCan::update(uint32_t now_ms)
{
    uint8_t valid = 0U;
    if ((control_seen != 0U) &&
        ((now_ms - last_control_ms) <= link_timeout_ms))
    {
        valid = 1U;
    }
    if ((heartbeat_seen != 0U) &&
        ((now_ms - last_heartbeat_ms) <= link_timeout_ms))
    {
        valid = 1U;
    }

    link_valid = valid;
    if (valid == 0U)
    {
        control_seen = 0U;
    }
}

} // namespace steering
