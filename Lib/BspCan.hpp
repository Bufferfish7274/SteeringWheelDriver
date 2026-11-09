#ifndef BSP_CAN_HPP
#define BSP_CAN_HPP

#include "main.h"

namespace steering::bsp_can
{

HAL_StatusTypeDef startAcceptAll(CAN_HandleTypeDef *hcan);

HAL_StatusTypeDef startMasterFilter(
    CAN_HandleTypeDef *hcan,
    uint32_t control_id,
    uint32_t heartbeat_id);

} // namespace steering::bsp_can

#endif
