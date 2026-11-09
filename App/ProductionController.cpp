#include "ProductionController.hpp"

#include <cmath>

#include "AngleOpt.hpp"
#include "AppConfig.hpp"
#include "WheelApp.hpp"

#if !WHEEL_BUILD_DEBUG

namespace steering
{

namespace
{

float wrap360(float angle_deg)
{
    angle_deg = std::fmod(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

} // namespace

void ProductionController::applyPendingControl(WheelApp &app)
{
    const float mapped_target_deg = wrap360(
        (config::MasterAngleSign *
         app.master.target_angle_deg) +
        config::MasterAngleOffsetDeg);

    const angle_opt::Result result = angle_opt::select(
        app.gm6020.angle_deg,
        mapped_target_deg,
        app.master.drive_erpm);

    if (app.wheel_active == 0U)
    {
        app.gm6020.enable();
        app.vesc.enable();
        app.target_ramp.reset(app.gm6020.angle_deg);
        app.wheel_active = 1U;
    }

    app.mapped_target_deg = mapped_target_deg;
    app.final_target_angle_deg = result.target_angle_deg;
    app.final_target_erpm = result.target_erpm;
    app.angle_reversed = result.reversed;

    app.target_ramp.setDesired(result.target_angle_deg);
    app.vesc.setTargetErpm(result.target_erpm);
    app.control_pending = 0U;
    app.need_fresh_control = 0U;
}

void ProductionController::step(WheelApp &app)
{
    app.master.update(app.ms);
    app.updateOnlineState();

    if (app.ms < config::SelfTestMs)
    {
        app.disableWheel();
        app.gm6020.controlStep(app.ms);
        app.vesc.controlStep(app.ms);
        return;
    }

    const bool master_ready =
        (app.master.link_valid != 0U) &&
        (app.master.control_seen != 0U);
    const bool actuators_online =
        (app.gm6020_online != 0U) &&
        (app.vesc_online != 0U);

    if (!master_ready || !actuators_online)
    {
        app.control_pending = 0U;
        app.disableWheel();
    }
    else
    {
        if (app.control_pending != 0U)
        {
            applyPendingControl(app);
        }

        if ((app.wheel_active != 0U) &&
            (app.need_fresh_control == 0U))
        {
            app.target_ramp.update(config::ControlDtS);
            app.gm6020.setTargetAngle(app.target_ramp.output());
        }
    }

    app.gm6020.controlStep(app.ms);
    app.vesc.controlStep(app.ms);
}

} // namespace steering

#endif
