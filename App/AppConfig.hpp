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

inline constexpr uint32_t kMasterWheelId = 1U;
inline constexpr uint32_t kGm6020Id = 1U;
inline constexpr uint32_t kGm6020ZeroRaw = 1386U;
inline constexpr int32_t kGm6020AngleSign = 1;
inline constexpr int32_t kGm6020VoltageSign = 1;
inline constexpr float kGm6020GearRatio = 1.0f;
inline constexpr uint32_t kVescControllerId = 67U;

inline constexpr float kMasterAngleSign = 1.0f;
inline constexpr float kMasterAngleOffsetDeg = 0.0f;

inline constexpr float kGm6020PosKp = 20.0f;
inline constexpr float kGm6020PosKi = 2.0f;
inline constexpr float kGm6020PosKd = 0.1f;
inline constexpr float kGm6020PosIntegralMax = 50.0f;
inline constexpr uint8_t kGm6020PosIntegralSeparate = 1U;
inline constexpr float kGm6020PosSeparationDeg = 10.0f;

inline constexpr float kGm6020SpeedKp = 70.0f;
inline constexpr float kGm6020SpeedKi = 50.0f;
inline constexpr float kGm6020SpeedKd = 0.0f;
inline constexpr float kGm6020SpeedIntegralMax = 8000.0f;
inline constexpr uint8_t kGm6020SpeedIntegralSeparate = 0U;
inline constexpr float kGm6020SpeedSeparationRpm = 100.0f;

inline constexpr float kControlDtS = 0.001f;
inline constexpr float kGm6020MaxSpeedRpm = 300.0f;
inline constexpr int32_t kGm6020VoltageLimitRaw = 20000;
inline constexpr uint32_t kGm6020OnlineTimeoutMs = 20U;
inline constexpr uint32_t kVescOnlineTimeoutMs = 100U;
inline constexpr uint32_t kMasterLinkTimeoutMs = 500U;

inline constexpr float kRampMaxSpeedDegS = 790.0f;
inline constexpr float kRampMaxAccelDegS2 = 4000.0f;

inline constexpr float kDebugDefaultAngleDeg = 0.0f;
inline constexpr int32_t kDebugDefaultDirectionErpm = 0;
inline constexpr float kDebugDefaultSpeedRpm = 50.0f;
inline constexpr int32_t kDebugDefaultVoltageRaw = 0;
inline constexpr int32_t kDebugDefaultTargetErpm = 0;
inline constexpr int32_t kDebugDefaultErpmLimit = 1000;
inline constexpr uint8_t kDebugDefaultUseRamp = 1U;
inline constexpr float kDebugDefaultAmplitudeDeg = 30.0f;
inline constexpr uint32_t kDebugDefaultPeriodMs = 4000U;
inline constexpr uint32_t kDebugMinPeriodMs = 200U;
inline constexpr float kDebugMaxAmplitudeDeg = 180.0f;

inline constexpr uint32_t kSelfTestMs = 500U;
inline constexpr uint32_t kTelemetryMs = 20U;
inline constexpr uint32_t kLedUpdateMs = 10U;
inline constexpr uint32_t kDebugHeartbeatMs = 500U;

inline constexpr uint16_t kLedMasterPin = GPIO_PIN_4;
inline constexpr uint16_t kLedVescPin = GPIO_PIN_5;
inline constexpr uint16_t kLedEnablePin = GPIO_PIN_6;
inline constexpr uint16_t kLedGm6020Pin = GPIO_PIN_7;

inline constexpr uint32_t kFaultMasterLink = 0x00000001U;
inline constexpr uint32_t kFaultNoControl = 0x00000002U;
inline constexpr uint32_t kFaultGm6020Offline = 0x00000004U;
inline constexpr uint32_t kFaultVescOffline = 0x00000008U;
inline constexpr uint32_t kFaultNeedFresh = 0x00000010U;
inline constexpr uint32_t kFaultGm6020Tx = 0x00000020U;
inline constexpr uint32_t kFaultVescTx = 0x00000040U;

} // namespace steering::config

#endif
