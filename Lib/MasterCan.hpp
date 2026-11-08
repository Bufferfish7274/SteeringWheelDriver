#ifndef MASTER_CAN_HPP
#define MASTER_CAN_HPP

#include <cstdint>

#include "main.h"

namespace steering
{

class MasterCan
{
public:
    struct Config
    {
        uint32_t wheel_id;
        uint32_t link_timeout_ms;
    };

    MasterCan() = default;
    MasterCan(const MasterCan &) = delete;
    MasterCan &operator=(const MasterCan &) = delete;

    void init(const Config &config);
    void onRx(const CAN_RxHeaderTypeDef *header,
              const uint8_t data[8],
              uint32_t now_ms);
    void update(uint32_t now_ms);

    static constexpr uint32_t kControlIdBase = 0x100U;
    static constexpr uint32_t kHeartbeatIdBase = 0x110U;

    uint8_t wheel_id;
    uint32_t control_id;
    uint32_t heartbeat_id;
    int32_t drive_erpm;
    uint16_t angle_code;
    float target_angle_deg;
    uint32_t last_control_ms;
    uint32_t last_heartbeat_ms;
    uint32_t link_timeout_ms;
    uint8_t control_seen;
    uint8_t heartbeat_seen;
    uint8_t link_valid;
};

} // namespace steering

#endif
