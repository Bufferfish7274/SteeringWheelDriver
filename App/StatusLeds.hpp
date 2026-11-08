#ifndef STATUS_LEDS_HPP
#define STATUS_LEDS_HPP

#include <cstdint>

namespace steering
{

class WheelApp;

class StatusLeds
{
public:
    void reset();
    void poll(WheelApp &app, uint32_t now_ms);

private:
    void update(WheelApp &app);
    uint32_t last_update_ms_;
};

} // namespace steering

#endif
