#ifndef VESC_HPP
#define VESC_HPP

#include <cstdint>

#include "main.h"

namespace steering
{

class Vesc
{
public:
    struct Config
    {
        uint32_t controller_id;
        uint32_t online_timeout_ms;
    };

    Vesc() = default;
    Vesc(const Vesc &) = delete;
    Vesc &operator=(const Vesc &) = delete;

    void init(CAN_HandleTypeDef *hcan, const Config &config);
    void onRx(const CAN_RxHeaderTypeDef *header,
              const uint8_t data[8],
              uint32_t now_ms);
    void setTargetErpm(int32_t erpm);
    void enable();
    void disable();
    void controlStep(uint32_t now_ms);
    HAL_StatusTypeDef sendErpm(int32_t erpm);

    static constexpr uint32_t kPacketSetRpm = 3U;
    static constexpr uint32_t kPacketStatus = 9U;

    CAN_HandleTypeDef *hcan;
    uint8_t controller_id;
    int32_t target_erpm;
    int32_t actual_erpm;
    float actual_current_a;
    float duty;
    uint32_t last_rx_ms;
    uint32_t online_timeout_ms;
    uint8_t is_online;
    uint8_t is_enabled;

    HAL_StatusTypeDef last_tx_status;
    uint32_t tx_ok_count;
    uint32_t tx_busy_count;
    uint32_t tx_error_count;
    uint32_t last_tx_id;
    int32_t last_tx_erpm;
    uint8_t last_tx_data[8];
};

} // namespace steering

#endif
