#ifndef PRODUCTION_CONTROLLER_HPP
#define PRODUCTION_CONTROLLER_HPP

namespace steering
{

class WheelApp;

class ProductionController
{
public:
    void step(WheelApp &app);

private:
    void applyPendingControl(WheelApp &app);
};

} // namespace steering

#endif
