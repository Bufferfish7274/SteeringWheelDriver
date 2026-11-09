#ifndef DEBUG_CONTROLLER_HPP
#define DEBUG_CONTROLLER_HPP

#include "WheelDebug.hpp"

namespace steering
{

class WheelApp;

class DebugController
{
public:
    void step(WheelApp &app);

private:
    static bool usesGm(WheelDebugMode mode);
    static void clearEnable(WheelApp &app);
    static float clampAbs(float value, float limit);
};

} // namespace steering

#endif
