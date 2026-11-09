#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <cstdint>

#include "main.h"

namespace steering
{

class WheelApp;

class Telemetry
{
public:
    void reset();
    void poll(WheelApp &app, uint32_t now_ms);
    void onTxComplete();

private:
    void send(WheelApp &app);

    volatile uint8_t busy_;
    uint32_t last_send_ms_;
    char buffer_[512];
};

} // namespace steering

#endif
