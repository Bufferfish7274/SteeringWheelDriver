#include "VESC.hpp"

#include <cstring>

namespace steering
{

void Vesc::init(CAN_HandleTypeDef *handle,
                const Config &config)
{
    if ((handle == nullptr) ||
        (config.controller_id > 0xFFU) ||
        (config.online_timeout_ms == 0U))
    {
        Error_Handler();
        return;
    }

    hcan = handle;
    controller_id = static_cast<uint8_t>(config.controller_id);
    target_erpm = 0;
    actual_erpm = 0;
    actual_current_a = 0.0f;
    duty = 0.0f;
    last_rx_ms = 0U;
    online_timeout_ms = config.online_timeout_ms;
    is_online = 0U;
    is_enabled = 0U;

    last_tx_status = HAL_ERROR;
    tx_ok_count = 0U;
    tx_busy_count = 0U;
    tx_error_count = 0U;
    last_tx_id = 0U;
    last_tx_erpm = 0;
    std::memset(last_tx_data, 0, sizeof(last_tx_data));
}

void Vesc::onRx(const CAN_RxHeaderTypeDef *header,
                const uint8_t data[8],
                uint32_t now_ms)
{
    if ((header == nullptr) || (data == nullptr))
    {
        return;
    }

    const uint32_t expected_id =
        (kPacketStatus << 8U) |
        static_cast<uint32_t>(controller_id);
    if ((header->IDE != CAN_ID_EXT) ||
        (header->RTR != CAN_RTR_DATA) ||
        (header->DLC != 8U) ||
        (header->ExtId != expected_id))
    {
        return;
    }

    const uint32_t erpm_raw =
        (static_cast<uint32_t>(data[0]) << 24U) |
        (static_cast<uint32_t>(data[1]) << 16U) |
        (static_cast<uint32_t>(data[2]) << 8U) |
        static_cast<uint32_t>(data[3]);
    const int16_t current_raw = static_cast<int16_t>(
        (static_cast<uint16_t>(data[4]) << 8U) |
        static_cast<uint16_t>(data[5]));
    const int16_t duty_raw = static_cast<int16_t>(
        (static_cast<uint16_t>(data[6]) << 8U) |
        static_cast<uint16_t>(data[7]));

    actual_erpm = static_cast<int32_t>(erpm_raw);
    actual_current_a =
        static_cast<float>(current_raw) / 10.0f;
    duty = static_cast<float>(duty_raw) / 1000.0f;
    last_rx_ms = now_ms;
    is_online = 1U;
}

void Vesc::setTargetErpm(int32_t erpm)
{
    target_erpm = erpm;
}

void Vesc::enable()
{
    target_erpm = 0;
    is_enabled = 1U;
}

void Vesc::disable()
{
    target_erpm = 0;
    is_enabled = 0U;
    sendErpm(0);
}

void Vesc::controlStep(uint32_t now_ms)
{
    if ((is_online != 0U) &&
        ((now_ms - last_rx_ms) > online_timeout_ms))
    {
        is_online = 0U;
    }

    if ((is_enabled == 0U) || (is_online == 0U))
    {
        sendErpm(0);
        return;
    }
    sendErpm(target_erpm);
}

HAL_StatusTypeDef Vesc::sendErpm(int32_t erpm)
{
    if (hcan == nullptr)
    {
        last_tx_status = HAL_ERROR;
        ++tx_error_count;
        return HAL_ERROR;
    }

    uint8_t data[8] = {};
    const uint32_t raw_erpm = static_cast<uint32_t>(erpm);
    data[0] = static_cast<uint8_t>(raw_erpm >> 24U);
    data[1] = static_cast<uint8_t>(raw_erpm >> 16U);
    data[2] = static_cast<uint8_t>(raw_erpm >> 8U);
    data[3] = static_cast<uint8_t>(raw_erpm);

    CAN_TxHeaderTypeDef header = {};
    header.ExtId =
        (kPacketSetRpm << 8U) |
        static_cast<uint32_t>(controller_id);
    header.StdId = 0U;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;

    last_tx_id = header.ExtId;
    last_tx_erpm = erpm;
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
