#ifndef STEERING_WHEEL_APP_H
#define STEERING_WHEEL_APP_H

#include <stdint.h>

#include "GM6020.h"
#include "VESC.h"
#include "angle_opt.h"
#include "app_config.h"
#include "setpoint_ramp.h"

#define APP_ACTUATOR_GM6020          0x01U
#define APP_ACTUATOR_VESC            0x02U
#define APP_ACTUATOR_ALL             (APP_ACTUATOR_GM6020 | APP_ACTUATOR_VESC)

#define APP_FAULT_MASTER_LINK        0x00000001U
#define APP_FAULT_NO_CONTROL         0x00000002U
#define APP_FAULT_GM6020_OFFLINE     0x00000004U
#define APP_FAULT_VESC_OFFLINE       0x00000008U
#define APP_FAULT_NEED_FRESH         0x00000010U
#define APP_FAULT_GM6020_TX          0x00000020U
#define APP_FAULT_VESC_TX            0x00000040U
#define APP_FAULT_CONFIG             0x00000080U

typedef enum
{
    APP_COMMAND_MOTION = 0,
    APP_COMMAND_GM6020_SPEED,
    APP_COMMAND_GM6020_VOLTAGE,
    APP_COMMAND_VESC_ERPM
} AppCommandKind_t;

typedef struct
{
    AppCommandKind_t kind;
    float wheel_target_angle_deg;
    int32_t drive_erpm;
    float gm6020_target_rpm;
    int16_t gm6020_target_voltage;
    int32_t vesc_target_erpm;
    uint8_t source_ready;
    uint8_t has_new_target;
    uint8_t required_actuators;
} AppInputSnapshot_t;

typedef struct
{
    uint8_t boot_complete;
    uint8_t control_enabled;
    uint8_t need_fresh_control;
    uint8_t gm6020_online;
    uint8_t vesc_online;
    uint8_t angle_reversed;
    uint8_t required_actuators;

    float mapped_target_deg;
    float final_target_angle_deg;
    int32_t final_target_erpm;

    uint32_t fault_flags;
    uint32_t applied_config_seq;
    AppConfigApplyStatus_t config_apply_status;
} AppSnapshot_t;

typedef HAL_StatusTypeDef (*AppStructuralApplyCallback_t)(
    void *context,
    const AppConfig_t *config);

typedef struct
{
    CAN_HandleTypeDef *can1;
    AppStructuralApplyCallback_t structural_apply;
    void *structural_context;

    AppConfig_t config;
    GM6020_t gm6020;
    VESC_t vesc;
    SetpointRamp_t ramp;

    uint8_t control_enabled;
    uint8_t need_fresh_control;
    uint8_t gm6020_online;
    uint8_t vesc_online;
    uint8_t angle_reversed;
    uint8_t required_actuators;

    float mapped_target_deg;
    float final_target_angle_deg;
    int32_t final_target_erpm;

    uint32_t last_now_ms;
    uint32_t applied_config_seq;
    AppConfigApplyStatus_t config_apply_status;
} SteeringWheelApp_t;

/** @brief 初始化共享控制核心和 GM6020/VESC 对象。 */
void SteeringWheelApp_Init(SteeringWheelApp_t *app,
                           CAN_HandleTypeDef *can1,
                           const AppConfig_t *config,
                           AppStructuralApplyCallback_t structural_apply,
                           void *structural_context);

/** @brief 把 CAN1 反馈分发给 GM6020 和 VESC。 */
void SteeringWheelApp_OnCan1Rx(SteeringWheelApp_t *app,
                               const CAN_RxHeaderTypeDef *header,
                               const uint8_t data[8],
                               uint32_t now_ms);

/**
 * @brief 执行一次应用控制。
 * @note 若新目标被接受或丢弃，会把 input->has_new_target 清零。
 */
void SteeringWheelApp_ControlStep(SteeringWheelApp_t *app,
                                  AppInputSnapshot_t *input,
                                  uint32_t now_ms);

/** @brief 立即失能并强制等待新目标。 */
void SteeringWheelApp_RequestDisable(SteeringWheelApp_t *app);

/** @brief 获取最近一次控制快照。 */
void SteeringWheelApp_GetSnapshot(const SteeringWheelApp_t *app,
                                  AppSnapshot_t *snapshot);

/** @brief 获取已生效配置，供命令源完成坐标映射。 */
const AppConfig_t *SteeringWheelApp_GetEffectiveConfig(
    const SteeringWheelApp_t *app);

/** @brief 将主控角映射到舵轮本地坐标并归一化。 */
float SteeringWheelApp_MapMasterAngle(const AppConfig_t *config,
                                      float master_angle_deg);

/** @brief 当前是否已使能。 */
uint8_t SteeringWheelApp_IsActive(const SteeringWheelApp_t *app);

#endif /* STEERING_WHEEL_APP_H */
