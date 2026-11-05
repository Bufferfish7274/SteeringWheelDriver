#ifndef APP_H
#define APP_H

/**
 * @file App.h
 * @brief 舵轮应用的顶层上下文和事件接口。
 *
 * App_t 把驱动对象、整轮安全状态、调度时间戳和遥测缓冲组合在一起。
 * main.c 只负责创建 App_t、调用生命周期函数，并把 HAL 中断事件转发给
 * App。具体运行路径由 AppConfig.h 中的 WHEEL_BUILD_DEBUG 决定：
 *
 * - WHEEL_BUILD_DEBUG=0：处理主控 CAN2 命令；
 * - WHEEL_BUILD_DEBUG=1：从 g_wheel_debug 生成调试目标，只启动 CAN1。
 *
 * 所有 App_* 控制函数必须在 TIM6 1 kHz 上下文中执行；CAN 和 UART
 * 回调只把事件交给 App，不在中断中执行阻塞操作。
 */

#include <stdint.h>

#include "AppConfig.h"
#include "GM6020.h"
#include "VESC.h"
#include "angle_opt.h"
#include "setpoint_ramp.h"

#if !WHEEL_BUILD_DEBUG
#include "MasterCan.h"
#endif

#if WHEEL_BUILD_DEBUG
#include "WheelDebug.h"
#endif

/**
 * @brief 一整轮应用的持久运行上下文。
 *
 * main.c 在静态存储区创建一个 App_t，ISR 和主循环共享同一个对象。
 * GM6020、VESC 和斜坡对象在生产和调试构建中始终存在；主控对象和调试
 * 运行状态则由 WHEEL_BUILD_DEBUG 条件编译，减少另一条路径的耦合。
 */
typedef struct
{
    /* CubeMX 初始化并启动应用所需的 HAL 句柄。 */
    CAN_HandleTypeDef *hcan1;
    TIM_HandleTypeDef *htim6;
    UART_HandleTypeDef *huart1;

    /* 本地电机总线和目标生成共用的驱动对象。 */
    GM6020_t gm6020;
    VESC_t vesc;
    SetpointRamp_t target_ramp;

#if !WHEEL_BUILD_DEBUG
    /* 主控总线和一次性新命令标志只在正式构建中存在。 */
    CAN_HandleTypeDef *hcan2;
    MasterCan_t master;
    volatile uint32_t control_pending;
#endif

    /* TIM6 维护的自由运行毫秒时钟，允许 uint32_t 自然回绕。 */
    volatile uint32_t ms;

    /* 应用层在线和整轮使能状态，统一供安全判断、LED 和遥测读取。 */
    volatile uint8_t gm6020_online;
    volatile uint8_t vesc_online;
    volatile uint8_t wheel_active;
    volatile uint8_t need_fresh_control;

    /* 最近一次目标转换结果，用于串口诊断和 Watch 观察。 */
    volatile uint8_t angle_reversed;
    volatile float mapped_target_deg;
    volatile float final_target_angle_deg;
    volatile int32_t final_target_erpm;

    /* UART 非阻塞发送状态和主循环调度时间戳。 */
    volatile uint8_t uart_busy;
    char uart_buffer[512];
    uint32_t last_telemetry_ms;
    uint32_t last_led_ms;

#if WHEEL_BUILD_DEBUG
    /* 波形相位、斜坡和 AngleOpt 选择结果只在调试构建中存在。 */
    WheelDebugRuntime_t debug_runtime;
#endif
} App_t;

/**
 * @brief 初始化整轮应用对象。
 * @param app 应用上下文；函数会先清空该对象。
 * @param hcan1 本地电机 CAN1 句柄。
 * @param hcan2 主控 CAN2 句柄；调试构建允许传入 NULL。
 * @param htim6 1 kHz 控制定时器句柄。
 * @param huart1 调试遥测 UART 句柄。
 *
 * 函数完成 GM6020/VESC/斜坡初始化、应用调参注入、输入源初始化和串口
 * 中断使能，但不启动 CAN 总线或 TIM6。
 */
void App_Init(App_t *app,
              CAN_HandleTypeDef *hcan1,
              CAN_HandleTypeDef *hcan2,
              TIM_HandleTypeDef *htim6,
              UART_HandleTypeDef *huart1);

/**
 * @brief 启动 CAN 接收和控制节拍。
 * @note 正式构建先启动 CAN1 和 CAN2，再启动 TIM6；调试构建不启动 CAN2。
 */
void App_Start(App_t *app);

/**
 * @brief 主循环低频调度入口。
 * @note 本函数只处理 LED 和 UART 遥测，不执行电机控制。
 */
void App_Loop(App_t *app);

/**
 * @brief 处理 CAN RX FIFO0 中断事件。
 *
 * CAN1 帧交给 GM6020 和 VESC 两个驱动自行过滤；正式构建中的 CAN2
 * 控制帧还会设置 control_pending，等待 TIM6 上下文统一应用。
 */
void App_OnCanRx(App_t *app, CAN_HandleTypeDef *hcan);

/**
 * @brief 处理定时器周期中断。
 *
 * 只有 app->htim6 对应实例会推进毫秒时钟和 1 kHz 控制步骤，从而避免
 * 将来增加其他定时器时误触发舵轮控制。
 */
void App_OnTimerElapsed(App_t *app, TIM_HandleTypeDef *htim);

/** @brief UART 发送完成后释放下一次遥测发送许可。 */
void App_OnUartTxComplete(App_t *app, UART_HandleTypeDef *huart);

#endif /* APP_H */
