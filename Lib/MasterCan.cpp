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
    const uint32_t group =
        (static_cast<uint32_t>(wheel_id) + 1U) / 2U;
    control_id = ControlIdBase + group;
    heartbeat_id = HeartbeatIdBase + group;
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
        const uint32_t slot_offset =
            ((static_cast<uint32_t>(wheel_id) - 1U) % 2U) * 4U;
        const uint16_t erpm_raw =
            static_cast<uint16_t>(
                (static_cast<uint16_t>(
                     data[slot_offset]) << 8U) |
                static_cast<uint16_t>(data[slot_offset + 1U]));
        uint16_t angle_raw =
            static_cast<uint16_t>(
                (static_cast<uint16_t>(
                     data[slot_offset + 2U]) << 8U) |
                static_cast<uint16_t>(data[slot_offset + 3U]));

        if (angle_raw > 16383U)
        {
            angle_raw = 16383U;
        }

        drive_erpm =
            ((erpm_raw & 0x8000U) != 0U)
                ? (static_cast<int32_t>(erpm_raw) - 65536)
                : static_cast<int32_t>(erpm_raw);
        angle_code = angle_raw;
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
