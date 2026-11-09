#ifndef APP_CONFIG_HPP
#define APP_CONFIG_HPP

#include <cstdint>

#include "main.h"

#ifndef WHEEL_BUILD_DEBUG
#define WHEEL_BUILD_DEBUG 1
#endif

#if (WHEEL_BUILD_DEBUG != 0) && (WHEEL_BUILD_DEBUG != 1)
#error "WHEEL_BUILD_DEBUG must be 0 or 1"
#endif

namespace steering::config
{

inline constexpr uint32_t MasterWheelId = 1U;
inline constexpr uint32_t Gm6020Id = 1U;
inline constexpr uint32_t Gm6020ZeroRaw = 1386U;
inline constexpr int32_t Gm6020AngleSign = 1;
inline constexpr int32_t Gm6020VoltageSign = 1;
inline constexpr float Gm6020GearRatio = 1.0f;
inline constexpr uint32_t VescControllerId = 67U;

inline constexpr float MasterAngleSign = 1.0f;
inline constexpr float MasterAngleOffsetDeg = 0.0f;

inline constexpr float Gm6020PosKp = 20.0f;
inline constexpr float Gm6020PosKi = 2.0f;
inline constexpr float Gm6020PosKd = 0.1f;
inline constexpr float Gm6020PosIntegralMax = 50.0f;
inline constexpr uint8_t Gm6020PosIntegralSeparate = 1U;
inline constexpr float Gm6020PosSeparationDeg = 10.0f;

inline constexpr float Gm6020SpeedKp = 70.0f;
inline constexpr float Gm6020SpeedKi = 50.0f;
inline constexpr float Gm6020SpeedKd = 0.0f;
inline constexpr float Gm6020SpeedIntegralMax = 8000.0f;
inline constexpr uint8_t Gm6020SpeedIntegralSeparate = 0U;
inline constexpr float Gm6020SpeedSeparationRpm = 100.0f;

inline constexpr float ControlDtS = 0.001f;
inline constexpr float Gm6020MaxSpeedRpm = 300.0f;
inline constexpr int32_t Gm6020VoltageLimitRaw = 20000;
inline constexpr uint32_t Gm6020OnlineTimeoutMs = 20U;
inline constexpr uint32_t VescOnlineTimeoutMs = 100U;
inline constexpr uint32_t MasterLinkTimeoutMs = 500U;

inline constexpr float RampMaxSpeedDegS = 790.0f;
inline constexpr float RampMaxAccelDegS2 = 4000.0f;

inline constexpr float DebugDefaultAngleDeg = 0.0f;
inline constexpr int32_t DebugDefaultDirectionErpm = 0;
inline constexpr float DebugDefaultSpeedRpm = 50.0f;
inline constexpr int32_t DebugDefaultVoltageRaw = 0;
inline constexpr int32_t DebugDefaultTargetErpm = 0;
inline constexpr int32_t DebugDefaultErpmLimit = 1000;
inline constexpr uint8_t DebugDefaultUseRamp = 1U;
inline constexpr float DebugDefaultAmplitudeDeg = 30.0f;
inline constexpr uint32_t DebugDefaultPeriodMs = 4000U;
inline constexpr uint32_t DebugMinPeriodMs = 200U;
inline constexpr float DebugMaxAmplitudeDeg = 180.0f;

inline constexpr uint32_t SelfTestMs = 500U;
inline constexpr uint32_t TelemetryMs = 20U;
inline constexpr uint32_t LedUpdateMs = 10U;
inline constexpr uint32_t DebugHeartbeatMs = 500U;

inline constexpr uint16_t LedMasterPin = GPIO_PIN_4;
inline constexpr uint16_t LedVescPin = GPIO_PIN_5;
inline constexpr uint16_t LedEnablePin = GPIO_PIN_6;
inline constexpr uint16_t LedGm6020Pin = GPIO_PIN_7;

inline constexpr uint32_t FaultMasterLink = 0x00000001U;
inline constexpr uint32_t FaultNoControl = 0x00000002U;
inline constexpr uint32_t FaultGm6020Offline = 0x00000004U;
inline constexpr uint32_t FaultVescOffline = 0x00000008U;
inline constexpr uint32_t FaultNeedFresh = 0x00000010U;
inline constexpr uint32_t FaultGm6020Tx = 0x00000020U;
inline constexpr uint32_t FaultVescTx = 0x00000040U;

} // namespace steering::config

#endif
