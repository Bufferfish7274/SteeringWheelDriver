#include <cmath>
#include <cstdint>
#include <cstdio>

#include "SetpointRamp.hpp"

static uint32_t g_failures;

static steering::SetpointRampConfig MakeConfig(
    float max_speed_deg_s,
    float max_accel_deg_s2)
{
    steering::SetpointRampConfig config = {};
    config.max_speed_deg_s = max_speed_deg_s;
    config.max_accel_deg_s2 = max_accel_deg_s2;

    return config;
}

static float Wrap180(float angle)
{
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static void Check(bool condition, const char *message)
{
    if (!condition)
    {
        std::printf("FAIL: %s\n", message);
        g_failures++;
    }
}

typedef struct
{
    float settle_time_s;
    float max_velocity;
    float max_velocity_step;
    float total_travel_deg;
} RampRun_t;

static RampRun_t RunRamp(steering::SetpointRamp &ramp,
                         float desired,
                         float dt,
                         uint32_t max_steps)
{
    RampRun_t run = {};
    float previous_velocity = ramp.velocity_deg_s;
    float previous_output = ramp.output_angle_deg;
    uint32_t step;

    ramp.setDesired(desired);

    for (step = 0U; step < max_steps; step++)
    {
        ramp.update(dt);

        if (std::fabs(ramp.velocity_deg_s) > run.max_velocity)
        {
            run.max_velocity = std::fabs(ramp.velocity_deg_s);
        }

        if (std::fabs(ramp.velocity_deg_s - previous_velocity) >
            run.max_velocity_step)
        {
            run.max_velocity_step =
                std::fabs(ramp.velocity_deg_s - previous_velocity);
        }

        run.total_travel_deg += Wrap180(
            ramp.output_angle_deg - previous_output);
        previous_velocity = ramp.velocity_deg_s;
        previous_output = ramp.output_angle_deg;

        if ((std::fabs(Wrap180(desired - ramp.output_angle_deg)) <
             0.05f) &&
            (std::fabs(ramp.velocity_deg_s) < 0.5f))
        {
            run.settle_time_s = (float)(step + 1U) * dt;
            return run;
        }
    }

    run.settle_time_s = -1.0f;
    return run;
}

static void Test_InitialAndReset(void)
{
    steering::SetpointRamp ramp;
    const steering::SetpointRampConfig config =
        MakeConfig(790.0f, 4000.0f);

    ramp.init(350.0f, config);
    Check(std::fabs(ramp.output_angle_deg - 350.0f) < 0.001f,
          "initial output angle");
    Check(std::fabs(ramp.desired_angle_deg - 350.0f) < 0.001f,
          "initial desired angle");
    Check(std::fabs(ramp.velocity_deg_s) < 0.001f,
          "initial velocity");

    ramp.reset(-10.0f);
    Check(std::fabs(ramp.output_angle_deg - 350.0f) < 0.001f,
          "reset output normalized");
    Check(std::fabs(ramp.velocity_deg_s) < 0.001f,
          "reset velocity");
}

static void Test_SmallStep(void)
{
    steering::SetpointRamp ramp;
    RampRun_t run;
    const steering::SetpointRampConfig config =
        MakeConfig(120.0f, 600.0f);

    ramp.init(0.0f, config);
    run = RunRamp(ramp, 5.0f, 0.001f, 2000U);

    Check(std::fabs(ramp.output_angle_deg - 5.0f) < 0.1f,
          "5 degree final angle");
    Check(run.settle_time_s > 0.0f, "5 degree settles");
    Check(run.settle_time_s < 0.4f, "5 degree settles under 400ms");
    Check(run.max_velocity <= 120.1f, "5 degree speed limit");
    Check(run.max_velocity_step <= 0.601f,
          "5 degree acceleration limit");
}

static void Test_MediumStep(void)
{
    steering::SetpointRamp ramp;
    RampRun_t run;
    const steering::SetpointRampConfig config =
        MakeConfig(120.0f, 600.0f);

    ramp.init(0.0f, config);
    run = RunRamp(ramp, 30.0f, 0.001f, 3000U);

    Check(std::fabs(ramp.output_angle_deg - 30.0f) < 0.1f,
          "30 degree final angle");
    Check(run.settle_time_s > 0.3f, "30 degree takes time");
    Check(run.settle_time_s < 0.7f, "30 degree settles under 700ms");
    Check(run.max_velocity <= 120.1f, "30 degree speed limit");
    Check(run.max_velocity_step <= 0.601f,
          "30 degree acceleration limit");
}

static void Test_LargeStep(void)
{
    steering::SetpointRamp ramp;
    RampRun_t run;
    const steering::SetpointRampConfig config =
        MakeConfig(120.0f, 600.0f);

    ramp.init(0.0f, config);
    run = RunRamp(ramp, 100.0f, 0.001f, 3000U);

    Check(std::fabs(ramp.output_angle_deg - 100.0f) < 0.1f,
          "100 degree final angle");
    Check(run.settle_time_s > 0.8f, "100 degree takes time");
    Check(run.settle_time_s < 1.4f, "100 degree settles under 1.4s");
    Check(run.max_velocity <= 120.1f, "100 degree speed limit");
    Check(run.max_velocity_step <= 0.601f,
          "100 degree acceleration limit");
}

static void Test_WrapShortestPath(void)
{
    steering::SetpointRamp ramp;
    RampRun_t run;
    const steering::SetpointRampConfig config =
        MakeConfig(120.0f, 600.0f);

    ramp.init(359.0f, config);
    run = RunRamp(ramp, 1.0f, 0.001f, 1000U);

    Check(std::fabs(Wrap180(ramp.output_angle_deg - 1.0f)) < 0.1f,
          "wrap final angle");
    Check(run.settle_time_s > 0.0f, "wrap settles");
    Check(run.total_travel_deg > 0.0f, "wrap takes short positive path");
    Check(std::fabs(run.total_travel_deg) < 3.0f,
          "wrap does not travel long way");
}

static void Test_TargetChange(void)
{
    steering::SetpointRamp ramp;
    RampRun_t run;
    const steering::SetpointRampConfig config =
        MakeConfig(120.0f, 600.0f);
    uint32_t i;

    ramp.init(0.0f, config);
    ramp.setDesired(90.0f);
    for (i = 0U; i < 300U; i++)
    {
        ramp.update(0.001f);
    }

    run = RunRamp(ramp, 0.0f, 0.001f, 3000U);
    Check(std::fabs(Wrap180(ramp.output_angle_deg)) < 0.1f,
          "target change returns to zero");
    Check(run.settle_time_s > 0.0f, "target change settles");
    Check(run.max_velocity_step <= 0.601f,
          "target change acceleration limit");
}

int main()
{
    Test_InitialAndReset();
    Test_SmallStep();
    Test_MediumStep();
    Test_LargeStep();
    Test_WrapShortestPath();
    Test_TargetChange();

    if (g_failures != 0U)
    {
        std::printf("SetpointRamp host tests: %u failure(s)\n",
                    static_cast<unsigned>(g_failures));
        return 1;
    }

    std::printf("SetpointRamp host tests: all passed\n");
    return 0;
}
