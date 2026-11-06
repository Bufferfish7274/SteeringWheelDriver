#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"

/**
 * @file bsp_can.h
 * @brief STM32 bxCAN 过滤器和启动流程的轻量封装。
 *
 * 本模块只负责：
 * - 配置接收过滤器；
 * - 启动 CAN；
 * - 开启 RX FIFO0 消息挂起通知。
 *
 * CAN 接收回调、报文解码和发送均由上层应用或其他驱动负责。
 * 所有配置遵循“先配置过滤器，再启动 CAN，最后开启中断通知”的顺序。
 */

/**
 * @brief 配置接收全部帧，然后启动指定 CAN 并开启 RX FIFO0 通知。
 * @param hcan 已由 CubeMX/HAL 初始化的 CAN 句柄。
 * @return HAL_OK 表示过滤器、启动和中断通知全部成功。
 * @note 用于本地电机总线等需要由软件再分发的 CAN 通道。
 */
HAL_StatusTypeDef BSP_CAN_StartAcceptAll(CAN_HandleTypeDef *hcan);

/**
 * @brief 只接收指定控制帧和心跳帧，然后启动 CAN。
 * @param hcan 已初始化的 CAN 句柄。
 * @param control_id 主控控制帧标准 ID。
 * @param heartbeat_id 主控心跳帧标准 ID。
 * @return HAL_OK 表示两个过滤器和 CAN 启动均成功。
 */
HAL_StatusTypeDef BSP_CAN_StartMasterFilter(
    CAN_HandleTypeDef *hcan,
    uint32_t control_id,
    uint32_t heartbeat_id);

#endif /* BSP_CAN_H */
