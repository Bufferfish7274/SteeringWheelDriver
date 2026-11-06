#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "angle_opt.h"

static uint32_t g_failures;

static void Check(int condition, const char *message)
{
    if (!condition)
    {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

static void CheckResult(float current,
                        float target,
                        int32_t erpm,
                        float expected_angle,
                        int32_t expected_erpm,
                        uint8_t expected_reversed,
                        const char *message)
{
    AngleOptResult_t result;

    AngleOpt_Select(current, target, erpm, &result);

    Check(fabsf(result.target_angle_deg - expected_angle) < 0.001f,
          message);
    Check(result.target_erpm == expected_erpm, message);
    Check(result.reversed == expected_reversed, message);
    Check((result.target_angle_deg >= 0.0f) &&
          (result.target_angle_deg < 360.0f),
          "output angle normalized");
}

static float Wrap180(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    if (angle_deg >= 180.0f)
    {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

static void Test_MaxRotation(void)
{
    const int32_t erpm_values[] = {-100, 0, 100};
    uint32_t current_deg;
    uint32_t target_deg;
    uint32_t erpm_index;

    for (current_deg = 0U; current_deg < 360U; current_deg++)
    {
        for (target_deg = 0U; target_deg < 360U; target_deg++)
        {
            for (erpm_index = 0U;
                 erpm_index <
                 (sizeof(erpm_values) / sizeof(erpm_values[0]));
                 erpm_index++)
            {
                AngleOptResult_t result;
                float travel_deg;

                AngleOpt_Select((float)current_deg,
                                (float)target_deg,
                                erpm_values[erpm_index],
                                &result);
                travel_deg = Wrap180(result.target_angle_deg -
                                     (float)current_deg);

                if (fabsf(travel_deg) > 90.001f)
                {
                    Check(0, "selected rotation exceeds 90 degrees");
                    return;
                }
            }
        }
    }
}

int main(void)
{
    CheckResult(0.0f, 0.0f, 100,
                0.0f, 100, 0U,
                "target 0 keeps direction");

    CheckResult(0.0f, 90.0f, 100,
                90.0f, 100, 0U,
                "90 degree tie keeps target");

    CheckResult(0.0f, 100.0f, 100,
                280.0f, -100, 1U,
                "100 degree target chooses reverse");

    CheckResult(359.0f, 1.0f, 100,
                1.0f, 100, 0U,
                "359 to 1 takes shortest path");

    CheckResult(1.0f, 359.0f, 100,
                359.0f, 100, 0U,
                "1 to 359 takes shortest path");

    CheckResult(0.0f, 180.0f, 100,
                0.0f, -100, 1U,
                "180 degree opposite is zero distance after reverse");

    CheckResult(0.0f, 100.0f, 0,
                280.0f, 0, 1U,
                "zero ERPM selects closer reverse angle");

    CheckResult(0.0f, 120.0f, 0,
                300.0f, 0, 1U,
                "zero ERPM turns 60 degrees instead of 120");

    CheckResult(60.0f, 120.0f, 0,
                120.0f, 0, 0U,
                "zero ERPM keeps closer original angle");

    CheckResult(30.0f, 120.0f, 0,
                120.0f, 0, 0U,
                "zero ERPM 90 degree tie keeps original");

    CheckResult(0.0f, 180.0f, 0,
                0.0f, 0, 1U,
                "zero ERPM opposite angle needs no rotation");

    CheckResult(-10.0f, 370.0f, 100,
                10.0f, 100, 0U,
                "negative and over-360 inputs normalize");

    CheckResult(720.0f, -360.0f, 100,
                0.0f, 100, 0U,
                "large angle inputs normalize");

    CheckResult(180.0f, 0.0f, 100,
                180.0f, -100, 1U,
                "opposite angle reverses ERPM");

    Test_MaxRotation();

    if (g_failures != 0U)
    {
        printf("AngleOpt host tests: %u failure(s)\n",
               (unsigned)g_failures);
        return 1;
    }

    printf("AngleOpt host tests: all passed\n");
    return 0;
}
