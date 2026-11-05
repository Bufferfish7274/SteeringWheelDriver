#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/**
 * @file AppConfig.h
 * @brief 舵轮应用的全部可调配置。
 *
 * 本文件把硬件身份、机械安装参数、控制增益、安全限幅和调试默认值集中
 * 在一处。更换舵轮、拆装电机或完成一轮调参后，应以本文件中的数值为准。
 *
 * 注意：
 * - 这里的 PID 参数在 App_Init() 时注入驱动；
 * - Watch 中临时修改不会写入 Flash，停机后仍需回填本文件；
 * - 调试完成并恢复正式功能时，必须重新检查通信和安全相关配置。
 */

#include "main.h"
#include "setpoint_ramp.h"

/*
 * 单一构建目标内的调试开关。
 *
 * 正式固件：0
 * 调试固件：1
 *
 * 修改此宏后重新编译并烧录同一 Keil Target。不要同时在 Keil 的
 * Preprocessor Symbols 中定义 WHEEL_BUILD_DEBUG，否则这里的修改会被覆盖。
 */
#ifndef WHEEL_BUILD_DEBUG
#define WHEEL_BUILD_DEBUG 1
#endif

/* 主控轮号、设备 ID 和机械安装配置。 */
#define APP_MASTER_WHEEL_ID              1U
#define APP_GM6020_ID                    1U
#define APP_GM6020_ZERO_RAW              1386U
#define APP_GM6020_ANGLE_SIGN            1
#define APP_GM6020_VOLTAGE_SIGN          1
#define APP_GM6020_GEAR_RATIO            1.0f
#define APP_VESC_CONTROLLER_ID           67U

#define APP_MASTER_ANGLE_SIGN            1.0f
#define APP_MASTER_ANGLE_OFFSET_DEG      0.0f

/*
 * GM6020 位置-速度串级 PID 参数。
 *
 * 外环把角度误差转换为目标 RPM，内环把 RPM 误差转换为电压 raw。
 * 位置环保留积分分离，速度环默认持续积分以克服低速静摩擦。
 */
#define APP_GM6020_POS_KP                20.0f
#define APP_GM6020_POS_KI                2.0f
#define APP_GM6020_POS_KD                0.1f
#define APP_GM6020_POS_INTEGRAL_MAX      50.0f
#define APP_GM6020_POS_INTEGRAL_SEPARATE 1U
#define APP_GM6020_POS_SEPARATION_DEG    10.0f

#define APP_GM6020_SPEED_KP              70.0f
#define APP_GM6020_SPEED_KI              50.0f
#define APP_GM6020_SPEED_KD              0.0f
#define APP_GM6020_SPEED_INTEGRAL_MAX    8000.0f
#define APP_GM6020_SPEED_INTEGRAL_SEPARATE 0U
#define APP_GM6020_SPEED_SEPARATION_RPM  100.0f

#define APP_GM6020_CONTROL_DT_S          0.001f
#define APP_GM6020_MAX_SPEED_RPM         300.0f
#define APP_GM6020_VOLTAGE_LIMIT_RAW     20000
#define APP_GM6020_ONLINE_TIMEOUT_MS     20U

/* GM6020 位置参考的速度和加速度上限。 */
#define APP_RAMP_MAX_SPEED_DEG_S         SETPOINT_RAMP_DEFAULT_MAX_SPEED_DEG_S
#define APP_RAMP_MAX_ACCEL_DEG_S2        SETPOINT_RAMP_DEFAULT_MAX_ACCEL_DEG_S2

/*
 * WHEEL_BUILD_DEBUG=1 时写入 g_wheel_debug 的初始值。
 * 所有运动模式仍要求手动把 enable 置 1，这些默认值不会自动使能电机。
 */
#define APP_DEBUG_DEFAULT_ANGLE_DEG      0.0f
#define APP_DEBUG_DEFAULT_DIRECTION_ERPM 0
#define APP_DEBUG_DEFAULT_SPEED_RPM      50.0f
#define APP_DEBUG_DEFAULT_VOLTAGE_RAW    0
#define APP_DEBUG_DEFAULT_TARGET_ERPM    0
#define APP_DEBUG_DEFAULT_ERPM_LIMIT     1000
#define APP_DEBUG_DEFAULT_USE_RAMP       1U
#define APP_DEBUG_DEFAULT_AMPLITUDE_DEG  30.0f
#define APP_DEBUG_DEFAULT_PERIOD_MS      4000U

/* 自检、LED 和串口遥测周期。 */
#define APP_SELF_TEST_MS                 500U
#define APP_TELEMETRY_MS                 20U
#define APP_LED_UPDATE_MS                10U

#define APP_LED_MASTER_PIN               GPIO_PIN_4
#define APP_LED_VESC_PIN                 GPIO_PIN_5
#define APP_LED_ENABLE_PIN               GPIO_PIN_6
#define APP_LED_GM6020_PIN               GPIO_PIN_7

/* UART 遥测故障位，多个故障可按位或同时出现。 */
#define APP_FAULT_MASTER_LINK            0x00000001U
#define APP_FAULT_NO_CONTROL             0x00000002U
#define APP_FAULT_GM6020_OFFLINE         0x00000004U
#define APP_FAULT_VESC_OFFLINE           0x00000008U
#define APP_FAULT_NEED_FRESH             0x00000010U
#define APP_FAULT_GM6020_TX              0x00000020U
#define APP_FAULT_VESC_TX                0x00000040U

#endif /* APP_CONFIG_H */
