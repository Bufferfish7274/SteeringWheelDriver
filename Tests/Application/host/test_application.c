#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "GM6020.h"
#include "MasterCan.h"
#include "bench_input.h"
#include "steering_wheel_app.h"

const int APP_STUB_CAN1 = 1;

static uint32_t g_failures;
static uint32_t g_tx_count;
static uint32_t g_structural_apply_count;
static HAL_StatusTypeDef g_structural_apply_result = HAL_OK;
static CAN_TxHeaderTypeDef g_last_tx_header;
static uint8_t g_last_tx_data[8];

static void Check(int condition, const char *message)
{
    if (!condition)
    {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

void Error_Handler(void)
{
    g_failures++;
}

HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    CAN_HandleTypeDef *hcan,
    CAN_TxHeaderTypeDef *header,
    uint8_t data[8],
    uint32_t *mailbox)
{
    (void)hcan;
    if ((header == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    g_last_tx_header = *header;
    memcpy(g_last_tx_data, data, 8U);
    g_tx_count++;
    if (mailbox != NULL)
    {
        *mailbox = 0U;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef Test_StructuralApply(
    void *context,
    const AppConfig_t *config)
{
    (void)context;
    (void)config;
    g_structural_apply_count++;
    return g_structural_apply_result;
}

static void Setup(SteeringWheelApp_t *app,
                  CAN_HandleTypeDef *can1,
                  AppConfig_t *config)
{
    AppConfig_LoadDefaults(config);
    AppConfig_InitTuning(config);
    SteeringWheelApp_Init(
        app,
        can1,
        config,
        Test_StructuralApply,
        NULL);
    g_tx_count = 0U;
    g_structural_apply_count = 0U;
    g_structural_apply_result = HAL_OK;
}

static void FeedGm6020(SteeringWheelApp_t *app,
                       uint32_t now_ms,
                       uint16_t raw_angle,
                       int16_t speed_rpm)
{
    CAN_RxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};

    header.StdId = 0x205U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    data[0] = (uint8_t)(raw_angle >> 8U);
    data[1] = (uint8_t)raw_angle;
    data[2] = (uint8_t)((uint16_t)speed_rpm >> 8U);
    data[3] = (uint8_t)speed_rpm;
    data[6] = 25U;

    SteeringWheelApp_OnCan1Rx(app, &header, data, now_ms);
}

static void FeedVesc(SteeringWheelApp_t *app,
                     uint32_t now_ms,
                     int32_t erpm)
{
    CAN_RxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint32_t raw = (uint32_t)erpm;

    header.ExtId = (9U << 8U) | 67U;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    data[0] = (uint8_t)(raw >> 24U);
    data[1] = (uint8_t)(raw >> 16U);
    data[2] = (uint8_t)(raw >> 8U);
    data[3] = (uint8_t)raw;

    SteeringWheelApp_OnCan1Rx(app, &header, data, now_ms);
}

static void FeedMasterFrame(MasterCan_t *master,
                            uint32_t id,
                            int32_t erpm,
                            uint16_t angle_code,
                            uint32_t now_ms)
{
    CAN_RxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint32_t raw_erpm = (uint32_t)erpm;

    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    data[0] = (uint8_t)(raw_erpm >> 24U);
    data[1] = (uint8_t)(raw_erpm >> 16U);
    data[2] = (uint8_t)(raw_erpm >> 8U);
    data[3] = (uint8_t)raw_erpm;
    data[4] = 0U;
    data[5] = 0U;
    data[6] = (uint8_t)(angle_code >> 8U);
    data[7] = (uint8_t)angle_code;

    MasterCan_OnRx(master, &header, data, now_ms);
}

static AppInputSnapshot_t InputFromMaster(const SteeringWheelApp_t *app,
                                          const MasterCan_t *master,
                                          uint8_t fresh)
{
    AppInputSnapshot_t input;

    memset(&input, 0, sizeof(input));
    input.kind = APP_COMMAND_MOTION;
    input.wheel_target_angle_deg = SteeringWheelApp_MapMasterAngle(
        SteeringWheelApp_GetEffectiveConfig(app),
        master->target_angle_deg);
    input.drive_erpm = master->drive_erpm;
    input.source_ready =
        (master->link_valid && master->control_seen) ? 1U : 0U;
    input.has_new_target = fresh;
    input.required_actuators = APP_ACTUATOR_ALL;
    return input;
}

static void FeedBoth(SteeringWheelApp_t *app,
                     uint32_t now_ms,
                     uint16_t raw_angle)
{
    FeedGm6020(app, now_ms, raw_angle, 0);
    FeedVesc(app, now_ms, 0);
}

static void Step(SteeringWheelApp_t *app,
                 AppInputSnapshot_t *input,
                 uint32_t now_ms)
{
    SteeringWheelApp_ControlStep(app, input, now_ms);
}

static AppInputSnapshot_t MotionInput(float angle,
                                      int32_t erpm,
                                      uint8_t source_ready,
                                      uint8_t fresh)
{
    AppInputSnapshot_t input;

    memset(&input, 0, sizeof(input));
    input.kind = APP_COMMAND_MOTION;
    input.wheel_target_angle_deg = angle;
    input.drive_erpm = erpm;
    input.source_ready = source_ready;
    input.has_new_target = fresh;
    input.required_actuators = APP_ACTUATOR_ALL;
    return input;
}

static void RunBoot(SteeringWheelApp_t *app,
                    AppInputSnapshot_t *input,
                    uint32_t end_ms,
                    uint16_t raw_angle)
{
    uint32_t now_ms;

    for (now_ms = 1U; now_ms <= end_ms; now_ms++)
    {
        FeedBoth(app, now_ms, raw_angle);
        Step(app, input, now_ms);
    }
}

static void Test_BootAndActivation(void)
{
    SteeringWheelApp_t app;
    CAN_HandleTypeDef can1 = {CAN1};
    AppConfig_t config;
    AppInputSnapshot_t input;
    AppSnapshot_t snapshot;
    uint16_t raw_angle = (uint16_t)(1386U + 228U);

    Setup(&app, &can1, &config);
    input = MotionInput(190.0f, 100, 1U, 1U);
    RunBoot(&app, &input, 499U, raw_angle);
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.boot_complete == 0U, "boot remains incomplete");
    Check(snapshot.control_enabled == 0U, "boot outputs disabled");

    FeedBoth(&app, 500U, raw_angle);
    Step(&app, &input, 500U);
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.boot_complete == 1U, "boot completes");
    Check(snapshot.control_enabled == 1U, "active after self test");
    Check(snapshot.angle_reversed == 1U, "AngleOpt reverses equivalent target");
    Check(snapshot.final_target_erpm == -100, "AngleOpt reverses ERPM");
    Check((snapshot.final_target_angle_deg < 11.0f) &&
          (snapshot.final_target_angle_deg > 9.0f),
          "selected equivalent angle");
    Check(input.has_new_target == 0U, "new target consumed once");
    Check(fabsf(app.ramp.output_angle_deg - 10.0f) < 0.2f,
          "ramp starts at current angle");

    FeedBoth(&app, 501U, (uint16_t)(1386U + 3868U));
    Step(&app, &input, 501U);
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.angle_reversed == 1U,
          "AngleOpt result remains stable without a new target");
    Check((snapshot.final_target_angle_deg < 11.0f) &&
          (snapshot.final_target_angle_deg > 9.0f),
          "AngleOpt is not repeated every control tick");
}

static void Test_FreshControlAndActuatorTimeout(void)
{
    SteeringWheelApp_t app;
    CAN_HandleTypeDef can1 = {CAN1};
    AppConfig_t config;
    AppInputSnapshot_t input;
    AppSnapshot_t snapshot;
    uint16_t raw_angle = (uint16_t)(1386U + 228U);
    uint32_t now_ms;

    Setup(&app, &can1, &config);
    input = MotionInput(30.0f, 0, 1U, 1U);
    RunBoot(&app, &input, 500U, raw_angle);
    Check(SteeringWheelApp_IsActive(&app) == 1U, "initial activation");

    input.source_ready = 0U;
    FeedBoth(&app, 501U, raw_angle);
    Step(&app, &input, 501U);
    Check(SteeringWheelApp_IsActive(&app) == 0U, "source loss disables");

    input.source_ready = 1U;
    FeedBoth(&app, 502U, raw_angle);
    Step(&app, &input, 502U);
    Check(SteeringWheelApp_IsActive(&app) == 0U,
          "source recovery alone does not resume");

    input.has_new_target = 1U;
    FeedBoth(&app, 503U, raw_angle);
    Step(&app, &input, 503U);
    Check(SteeringWheelApp_IsActive(&app) == 1U,
          "fresh target resumes control");

    FeedBoth(&app, 504U, raw_angle);
    Step(&app, &input, 504U);
    for (now_ms = 505U; now_ms <= 524U; now_ms++)
    {
        Step(&app, &input, now_ms);
    }
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.gm6020_online == 1U, "GM6020 online at 20 ms");
    Check(snapshot.control_enabled == 1U, "GM6020 20 ms remains active");

    Step(&app, &input, 525U);
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.gm6020_online == 0U, "GM6020 offline after 21 ms");
    Check(snapshot.control_enabled == 0U, "GM6020 timeout disables wheel");
    Check((snapshot.fault_flags & APP_FAULT_GM6020_OFFLINE) != 0U,
          "GM6020 fault flag");

    input.has_new_target = 1U;
    FeedBoth(&app, 526U, raw_angle);
    Step(&app, &input, 526U);
    Check(SteeringWheelApp_IsActive(&app) == 1U,
          "fresh command recovers after actuator timeout");

    for (now_ms = 527U; now_ms <= 626U; now_ms++)
    {
        FeedGm6020(&app, now_ms, raw_angle, 0);
        Step(&app, &input, now_ms);
    }
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.vesc_online == 1U, "VESC online at 100 ms");
    Step(&app, &input, 627U);
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.vesc_online == 0U, "VESC offline after 101 ms");
    Check(snapshot.control_enabled == 0U, "VESC timeout disables wheel");
}

static void Test_ConfigApplication(void)
{
    SteeringWheelApp_t app;
    CAN_HandleTypeDef can1 = {CAN1};
    AppConfig_t config;
    AppInputSnapshot_t input;
    AppSnapshot_t snapshot;
    uint16_t raw_angle = (uint16_t)(1386U + 228U);

    Setup(&app, &can1, &config);
    input = MotionInput(30.0f, 0, 1U, 1U);
    RunBoot(&app, &input, 500U, raw_angle);
    Check(SteeringWheelApp_IsActive(&app) == 1U, "config test active");

    g_tuning_config.position_pid.kp = 35.0f;
    g_tuning_apply_seq++;
    FeedBoth(&app, 501U, raw_angle);
    Step(&app, &input, 501U);
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.config_apply_status == APP_CONFIG_APPLY_OK,
          "live config accepted");
    Check(fabsf(app.gm6020.position_pid.kp - 35.0f) < 0.001f,
          "live PID applied");
    Check(SteeringWheelApp_IsActive(&app) == 1U,
          "live PID does not interrupt control");

    g_tuning_config.gm6020_id = 2U;
    g_tuning_apply_seq++;
    FeedBoth(&app, 502U, raw_angle);
    Step(&app, &input, 502U);
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.config_apply_status ==
          APP_CONFIG_APPLY_REJECTED_ACTIVE,
          "structural config rejected while active");
    Check(g_structural_apply_count == 0U,
          "rejected structural config does not reconfigure bus");

    SteeringWheelApp_RequestDisable(&app);
    g_tuning_apply_seq++;
    FeedBoth(&app, 503U, raw_angle);
    input.source_ready = 1U;
    input.has_new_target = 1U;
    Step(&app, &input, 503U);
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.config_apply_status == APP_CONFIG_APPLY_OK,
          "structural config accepted while disabled");
    Check(g_structural_apply_count == 1U,
          "structural config reconfigures bus once");
    Check(app.gm6020.id == 2U, "structural GM6020 ID applied");

    g_tuning_config.position_pid.kp = -1.0f;
    g_tuning_apply_seq++;
    FeedBoth(&app, 504U, raw_angle);
    Step(&app, &input, 504U);
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.config_apply_status == APP_CONFIG_APPLY_INVALID,
          "invalid config rejected");
    Check(app.gm6020.position_pid.kp > 0.0f,
          "invalid config preserves effective parameters");

    app.gm6020.position_pid.integral = 12.0f;
    g_tuning_config.position_pid.kp = 21.0f;
    g_tuning_reset_controllers = 1U;
    g_tuning_apply_seq++;
    FeedBoth(&app, 505U, raw_angle);
    Step(&app, &input, 505U);
    Check(fabsf(app.gm6020.position_pid.integral) < 0.001f,
          "explicit apply resets PID state");
    Check(g_tuning_reset_controllers == 0U,
          "controller reset request consumed");

    g_structural_apply_result = HAL_ERROR;
    g_tuning_config.gm6020_id = 3U;
    g_tuning_apply_seq++;
    FeedBoth(&app, 506U, raw_angle);
    Step(&app, &input, 506U);
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.config_apply_status == APP_CONFIG_APPLY_BUS_ERROR,
          "bus reconfiguration error reported");
    Check(SteeringWheelApp_IsActive(&app) == 0U,
          "bus reconfiguration error keeps outputs disabled");
}

static void Test_MasterCanLifecycle(void)
{
    SteeringWheelApp_t app;
    MasterCan_t master;
    CAN_HandleTypeDef can1 = {CAN1};
    AppConfig_t config;
    AppInputSnapshot_t input;
    uint16_t raw_angle = (uint16_t)(1386U + 228U);
    uint32_t now_ms;
    uint8_t fresh = 1U;
    uint8_t active_seen = 0U;

    Setup(&app, &can1, &config);
    MasterCan_Init(&master, 1U);
    FeedMasterFrame(&master, 0x101U, 0, 16384U / 4U, 1U);

    for (now_ms = 1U; now_ms <= 1102U; now_ms++)
    {
        if ((now_ms <= 600U) &&
            (((now_ms - 1U) % 80U) == 0U))
        {
            FeedMasterFrame(&master, 0x111U, 0, 0U, now_ms);
        }
        MasterCan_Update(&master, now_ms);
        FeedBoth(&app, now_ms, raw_angle);
        input = InputFromMaster(&app, &master, fresh);
        Step(&app, &input, now_ms);
        if (input.has_new_target == 0U)
        {
            fresh = 0U;
        }
        if (SteeringWheelApp_IsActive(&app))
        {
            active_seen = 1U;
        }
    }

    Check(active_seen == 1U, "MasterCan command activates application");
    Check(SteeringWheelApp_IsActive(&app) == 0U,
          "500 ms master link timeout disables application");

    FeedMasterFrame(&master, 0x111U, 0, 0U, 1103U);
    MasterCan_Update(&master, 1103U);
    FeedBoth(&app, 1103U, raw_angle);
    input = InputFromMaster(&app, &master, 0U);
    Step(&app, &input, 1103U);
    Check(SteeringWheelApp_IsActive(&app) == 0U,
          "heartbeat alone does not restore old target");

    FeedMasterFrame(&master, 0x101U, 0, 0U, 1104U);
    MasterCan_Update(&master, 1104U);
    FeedBoth(&app, 1104U, raw_angle);
    input = InputFromMaster(&app, &master, 1U);
    Step(&app, &input, 1104U);
    Check(SteeringWheelApp_IsActive(&app) == 1U,
          "new master control frame restores application");
}

#ifdef APP_BENCH
static void Test_BenchPositionManual(void)
{
    SteeringWheelApp_t app;
    CAN_HandleTypeDef can1 = {CAN1};
    AppConfig_t config;
    AppInputSnapshot_t input;
    AppSnapshot_t snapshot;
    uint16_t raw_angle = (uint16_t)(1386U + 228U);
    uint32_t now_ms;

    Setup(&app, &can1, &config);
    BenchInput_Reset(&config);
    g_bench_input.mode = APP_BENCH_POSITION_MANUAL;
    g_bench_input.enable = 1U;
    g_bench_input.target_angle_deg = 30.0f;
    g_bench_input.target_erpm = 0;

    for (now_ms = 1U; now_ms <= 500U; now_ms++)
    {
        FeedGm6020(&app, now_ms, raw_angle, 0);
        BenchInput_Build(
            SteeringWheelApp_GetEffectiveConfig(&app),
            app.gm6020.angle_deg,
            &input,
            now_ms);
        Step(&app, &input, now_ms);
        if (input.has_new_target == 0U)
        {
            BenchInput_TargetConsumed();
        }
    }

    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.control_enabled == 1U,
          "manual position target survives boot delay");
    Check(fabsf(snapshot.final_target_angle_deg - 30.0f) < 0.01f,
          "manual position enters shared control pipeline");
    Check(snapshot.angle_reversed == 0U,
          "zero ERPM keeps original position direction");
}

static void Test_BenchActuatorMasks(void)
{
    SteeringWheelApp_t app;
    CAN_HandleTypeDef can1 = {CAN1};
    AppConfig_t config;
    AppInputSnapshot_t input;
    AppSnapshot_t snapshot;
    uint16_t raw_angle = (uint16_t)(1386U + 228U);
    uint32_t now_ms;

    Setup(&app, &can1, &config);
    BenchInput_Reset(&config);

    g_bench_input.mode = APP_BENCH_GM6020_SPEED;
    g_bench_input.enable = 1U;
    g_bench_input.target_rpm = 50.0f;
    for (now_ms = 1U; now_ms <= 499U; now_ms++)
    {
        FeedGm6020(&app, now_ms, raw_angle, 0);
        BenchInput_Build(
            SteeringWheelApp_GetEffectiveConfig(&app),
            app.gm6020.angle_deg,
            &input,
            now_ms);
        Step(&app, &input, now_ms);
    }
    FeedGm6020(&app, 500U, raw_angle, 0);
    BenchInput_Build(
        SteeringWheelApp_GetEffectiveConfig(&app),
        app.gm6020.angle_deg,
        &input,
        500U);
    Step(&app, &input, 500U);
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.control_enabled == 1U,
          "GM speed bench starts with GM only");

    g_bench_input.mode = APP_BENCH_GM6020_VOLTAGE;
    FeedGm6020(&app, 501U, raw_angle, 0);
    BenchInput_Build(
        SteeringWheelApp_GetEffectiveConfig(&app),
        app.gm6020.angle_deg,
        &input,
        501U);
    Step(&app, &input, 501U);
    Check(SteeringWheelApp_IsActive(&app) == 0U,
          "mode change requires rearm");

    g_bench_input.enable = 0U;
    BenchInput_Build(
        SteeringWheelApp_GetEffectiveConfig(&app),
        app.gm6020.angle_deg,
        &input,
        502U);
    g_bench_input.enable = 1U;
    g_bench_input.target_voltage = 1000;
    FeedGm6020(&app, 502U, raw_angle, 0);
    BenchInput_Build(
        SteeringWheelApp_GetEffectiveConfig(&app),
        app.gm6020.angle_deg,
        &input,
        502U);
    Step(&app, &input, 502U);
    Check(SteeringWheelApp_IsActive(&app) == 1U,
          "voltage mode starts after explicit rearm");
    Check(app.gm6020.target_voltage == 1000,
          "direct voltage reaches GM6020");

    g_bench_input.mode = APP_BENCH_DISABLED;
    g_bench_input.enable = 0U;
    BenchInput_Build(
        SteeringWheelApp_GetEffectiveConfig(&app),
        app.gm6020.angle_deg,
        &input,
        503U);
    Step(&app, &input, 503U);
    g_bench_input.mode = APP_BENCH_VESC_ERPM;
    g_bench_input.enable = 1U;
    g_bench_input.target_erpm = 1000;
    for (now_ms = 504U; now_ms <= 506U; now_ms++)
    {
        FeedVesc(&app, now_ms, 0);
        BenchInput_Build(
            SteeringWheelApp_GetEffectiveConfig(&app),
            app.gm6020.angle_deg,
            &input,
            now_ms);
        Step(&app, &input, now_ms);
    }
    SteeringWheelApp_GetSnapshot(&app, &snapshot);
    Check(snapshot.control_enabled == 1U,
          "VESC bench requires VESC only");
    Check(app.vesc.target_erpm == 1000,
          "VESC bench target applied");
}
#endif

int main(void)
{
    Test_BootAndActivation();
    Test_FreshControlAndActuatorTimeout();
    Test_ConfigApplication();
    Test_MasterCanLifecycle();
#ifdef APP_BENCH
    Test_BenchPositionManual();
    Test_BenchActuatorMasks();
    printf("Application host tests (bench): %u failure(s)\n",
           (unsigned)g_failures);
#else
    printf("Application host tests (production): %u failure(s)\n",
           (unsigned)g_failures);
#endif

    return (g_failures == 0U) ? 0 : 1;
}
