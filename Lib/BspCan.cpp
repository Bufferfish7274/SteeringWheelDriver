#include "BspCan.hpp"

namespace steering::bsp_can
{

namespace
{

constexpr uint32_t Can2FirstFilterBank = 14U;
constexpr uint32_t StandardIdMask = 0x7FFU;

uint32_t firstFilterBank(const CAN_HandleTypeDef *hcan)
{
    return (hcan->Instance == CAN2) ? Can2FirstFilterBank : 0U;
}

HAL_StatusTypeDef configureStdFilter(
    CAN_HandleTypeDef *hcan,
    uint32_t filter_bank,
    uint32_t std_id)
{
    CAN_FilterTypeDef filter = {};

    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = static_cast<uint16_t>(std_id << 5U);
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh =
        static_cast<uint16_t>(StandardIdMask << 5U);
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = Can2FirstFilterBank;

    return HAL_CAN_ConfigFilter(hcan, &filter);
}

HAL_StatusTypeDef startWithNotification(
    CAN_HandleTypeDef *hcan)
{
    HAL_StatusTypeDef status = HAL_CAN_Start(hcan);
    if (status != HAL_OK)
    {
        return status;
    }
    return HAL_CAN_ActivateNotification(
        hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING);
}

} // namespace

HAL_StatusTypeDef startAcceptAll(CAN_HandleTypeDef *hcan)
{
    if (hcan == nullptr)
    {
        return HAL_ERROR;
    }

    CAN_FilterTypeDef filter = {};
    filter.FilterBank = firstFilterBank(hcan);
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = Can2FirstFilterBank;

    const HAL_StatusTypeDef status =
        HAL_CAN_ConfigFilter(hcan, &filter);
    if (status != HAL_OK)
    {
        return status;
    }
    return startWithNotification(hcan);
}

HAL_StatusTypeDef startMasterFilter(
    CAN_HandleTypeDef *hcan,
    uint32_t control_id,
    uint32_t heartbeat_id)
{
    if ((hcan == nullptr) ||
        (control_id > StandardIdMask) ||
        (heartbeat_id > StandardIdMask))
    {
        return HAL_ERROR;
    }

    const uint32_t first_bank = firstFilterBank(hcan);
    HAL_StatusTypeDef status = configureStdFilter(
        hcan,
        first_bank,
        control_id);
    if (status != HAL_OK)
    {
        return status;
    }

    status = configureStdFilter(
        hcan,
        first_bank + 1U,
        heartbeat_id);
    if (status != HAL_OK)
    {
        return status;
    }
    return startWithNotification(hcan);
}

} // namespace steering::bsp_can
