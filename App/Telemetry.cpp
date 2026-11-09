#include "Telemetry.hpp"

#include <cstdio>

#include "AppConfig.hpp"
#include "WheelApp.hpp"

namespace steering
{

namespace
{

uint32_t elapsedMs(uint32_t now_ms, uint32_t timestamp_ms)
{
    return static_cast<uint32_t>(now_ms - timestamp_ms);
}

#if !WHEEL_BUILD_DEBUG
uint32_t ageMs(uint32_t now_ms, uint32_t timestamp_ms)
{
    return (timestamp_ms == 0U)
               ? 0xFFFFFFFFU
               : elapsedMs(now_ms, timestamp_ms);
}
#endif

} // namespace

void Telemetry::reset()
{
    busy_ = 0U;
    last_send_ms_ = 0U;
    buffer_[0] = '\0';
}

void Telemetry::onTxComplete()
{
    busy_ = 0U;
}

void Telemetry::poll(WheelApp &app, uint32_t now_ms)
{
    if ((busy_ != 0U) ||
        (elapsedMs(now_ms, last_send_ms_) <
         config::TelemetryMs))
    {
        return;
    }

    last_send_ms_ = now_ms;
    send(app);
}

void Telemetry::send(WheelApp &app)
{
    uint8_t master_link = 0U;
    uint8_t control_seen = 0U;
    uint8_t need_fresh = 0U;
    uint32_t control_age_ms = 0xFFFFFFFFU;
    uint32_t heartbeat_age_ms = 0xFFFFFFFFU;

#if !WHEEL_BUILD_DEBUG
    master_link = app.master.link_valid;
    control_seen = app.master.control_seen;
    need_fresh = app.need_fresh_control;
    control_age_ms = ageMs(app.ms, app.master.last_control_ms);
    heartbeat_age_ms =
        ageMs(app.ms, app.master.last_heartbeat_ms);
#endif

    int length = std::snprintf(
        buffer_,
        sizeof(buffer_),
        "%u,%u,%u,%u,%u,%u,%u,%u,"
        "%d,%u,%d,%d,%d,%d,"
        "%d,%d,%d,%u,%d,"
        "%d,%d,%d,%d,%d,"
        "%u,%u,%u",
        static_cast<unsigned>(app.ms),
        static_cast<unsigned>(master_link),
        static_cast<unsigned>(control_seen),
        static_cast<unsigned>(app.gm6020_online),
        static_cast<unsigned>(app.vesc_online),
        static_cast<unsigned>(app.wheel_active),
        static_cast<unsigned>(app.angle_reversed),
        static_cast<unsigned>(need_fresh),
#if WHEEL_BUILD_DEBUG
        0,
#else
        static_cast<int>(app.master.drive_erpm),
#endif
#if WHEEL_BUILD_DEBUG
        0U,
#else
        static_cast<unsigned>(app.master.angle_code),
#endif
#if WHEEL_BUILD_DEBUG
        0,
#else
        static_cast<int>(app.master.target_angle_deg * 100.0f),
#endif
        static_cast<int>(app.mapped_target_deg * 100.0f),
        static_cast<int>(app.final_target_angle_deg * 100.0f),
        static_cast<int>(app.final_target_erpm),
        static_cast<int>(app.gm6020.angle_deg * 100.0f),
        static_cast<int>(app.gm6020.speed_rpm),
        static_cast<int>(app.gm6020.current_raw),
        static_cast<unsigned>(app.gm6020.temperature),
        static_cast<int>(app.gm6020.target_voltage),
        static_cast<int>(app.vesc.actual_erpm),
        static_cast<int>(app.vesc.actual_current_a * 1000.0f),
        static_cast<int>(app.vesc.duty * 1000.0f),
        static_cast<int>(app.target_ramp.output() * 100.0f),
        static_cast<int>(app.target_ramp.velocity()),
        static_cast<unsigned>(control_age_ms),
        static_cast<unsigned>(heartbeat_age_ms),
        static_cast<unsigned>(app.faultFlags()));

#if !WHEEL_BUILD_DEBUG
    if ((length > 0) &&
        (static_cast<uint32_t>(length) < sizeof(buffer_)))
    {
        const int ending_length = std::snprintf(
            buffer_ + length,
            sizeof(buffer_) - static_cast<uint32_t>(length),
            "\r\n");
        if ((ending_length <= 0) ||
            (static_cast<uint32_t>(ending_length) >=
             (sizeof(buffer_) - static_cast<uint32_t>(length))))
        {
            return;
        }
        length += ending_length;
    }
#endif

#if WHEEL_BUILD_DEBUG
    if ((length > 0) &&
        (static_cast<uint32_t>(length) < sizeof(buffer_)))
    {
        const int extra_length = std::snprintf(
            buffer_ + length,
            sizeof(buffer_) - static_cast<uint32_t>(length),
            ",%u,%u,%d,%d,%d,%u,%d,"
            "%d,%d,%d,%d,%d,%d,"
            "%u,%u,%u,%d\r\n",
            static_cast<unsigned>(g_wheel_debug.mode),
            static_cast<unsigned>(g_wheel_debug.enable),
            static_cast<int>(g_wheel_debug.target_angle_deg * 100.0f),
            static_cast<int>(g_wheel_debug.direction_erpm),
            static_cast<int>(app.debug_runtime.source_angle_deg * 100.0f),
            static_cast<unsigned>(g_wheel_debug.use_ramp),
            static_cast<int>(app.gm6020.target_speed_rpm),
            static_cast<int>(app.gm6020.position_pid.proportional * 100.0f),
            static_cast<int>(app.gm6020.position_pid.integral * 100.0f),
            static_cast<int>(app.gm6020.position_pid.differential * 100.0f),
            static_cast<int>(app.gm6020.speed_pid.proportional * 100.0f),
            static_cast<int>(app.gm6020.speed_pid.integral * 100.0f),
            static_cast<int>(app.gm6020.speed_pid.differential * 100.0f),
            static_cast<unsigned>(HAL_CAN_GetState(app.hcan1)),
            static_cast<unsigned>(HAL_CAN_GetError(app.hcan1)),
            static_cast<unsigned>(
                HAL_CAN_GetTxMailboxesFreeLevel(app.hcan1)),
            static_cast<int>(g_wheel_debug.erpm_limit));
        if ((extra_length <= 0) ||
            (static_cast<uint32_t>(extra_length) >=
             (sizeof(buffer_) - static_cast<uint32_t>(length))))
        {
            return;
        }
        length += extra_length;
    }
#endif

    if ((length > 0) &&
        (static_cast<uint32_t>(length) < sizeof(buffer_)))
    {
        if (HAL_UART_Transmit_IT(
                app.huart1,
                reinterpret_cast<uint8_t *>(buffer_),
                static_cast<uint16_t>(length)) == HAL_OK)
        {
            busy_ = 1U;
        }
    }
}

} // namespace steering
