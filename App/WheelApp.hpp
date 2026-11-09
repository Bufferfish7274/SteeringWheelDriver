#ifndef WHEEL_APP_HPP
#define WHEEL_APP_HPP

#include <cstdint>

#include "AppConfig.hpp"
#include "GM6020.hpp"
#include "SetpointRamp.hpp"
#include "StatusLeds.hpp"
#include "Telemetry.hpp"
#include "VESC.hpp"

#if !WHEEL_BUILD_DEBUG
#include "MasterCan.hpp"
#include "ProductionController.hpp"
#else
#include "DebugController.hpp"
#include "WheelDebug.hpp"
#endif

namespace steering
{

class WheelApp
{
public:
    WheelApp() = default;
    WheelApp(const WheelApp &) = delete;
    WheelApp &operator=(const WheelApp &) = delete;

    void init(CAN_HandleTypeDef *hcan1,
              CAN_HandleTypeDef *hcan2,
              TIM_HandleTypeDef *htim6,
              UART_HandleTypeDef *huart1);
    void start();
    void loop();
    void onCanRx(CAN_HandleTypeDef *hcan);
    void onTimerElapsed(TIM_HandleTypeDef *htim);
    void onUartTxComplete(UART_HandleTypeDef *huart);

    CAN_HandleTypeDef *hcan1;
    TIM_HandleTypeDef *htim6;
    UART_HandleTypeDef *huart1;

    Gm6020 gm6020;
    Vesc vesc;
    SetpointRamp target_ramp;

#if !WHEEL_BUILD_DEBUG
    MasterCan master;
    volatile uint32_t control_pending;
#endif

    volatile uint32_t ms;
    volatile uint8_t gm6020_online;
    volatile uint8_t vesc_online;
    volatile uint8_t wheel_active;
    volatile uint8_t need_fresh_control;
    volatile uint8_t angle_reversed;
    volatile float mapped_target_deg;
    volatile float final_target_angle_deg;
    volatile int32_t final_target_erpm;

#if WHEEL_BUILD_DEBUG
    WheelDebugRuntime debug_runtime;
#endif

private:
    friend class ProductionController;
    friend class DebugController;
    friend class Telemetry;

    void controlStep();
    void disableActuators();
    void disableWheel();
    void updateOnlineState();
    uint32_t faultFlags() const;

    CAN_HandleTypeDef *hcan2_;
#if !WHEEL_BUILD_DEBUG
    ProductionController production_controller_;
#else
    DebugController debug_controller_;
#endif
    Telemetry telemetry_;
    StatusLeds leds_;
};

} // namespace steering

extern steering::WheelApp g_app;

#endif
