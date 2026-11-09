#include <cmath>
#include <cstdio>
#include <cstring>

#include "BspCan.hpp"
#include "GM6020.hpp"

const int GM6020_STUB_CAN1 = 1;
const int GM6020_STUB_CAN2 = 2;

static CAN_TxHeaderTypeDef g_last_tx_header;
static uint8_t g_last_tx_data[8];
static uint32_t g_tx_count;
static CAN_FilterTypeDef g_filters[4];
static uint32_t g_filter_count;
static uint32_t g_start_count;
static uint32_t g_notify_count;
static char g_call_order[16];
static uint32_t g_call_order_len;

static uint32_t g_failures;

static steering::Gm6020::Config MakeConfig()
{
    steering::Gm6020::Config config = {};
    config.id = 1U;
    config.zero_raw = 0U;
    config.angle_sign = 1;
    config.voltage_sign = 1;
    config.gear_ratio = 1.0f;
    config.position =
    {
        20.0f, 2.0f, 0.1f, 50.0f, 1U, 10.0f
    };
    config.speed =
    {
        70.0f, 50.0f, 0.0f, 8000.0f, 0U, 100.0f
    };
    config.control_dt_s = 0.001f;
    config.max_speed_rpm = 300.0f;
    config.voltage_limit_raw = 20000;
    config.online_timeout_ms = 20U;

    return config;
}

static void InitMotor(steering::Gm6020 &motor,
                      CAN_HandleTypeDef *hcan,
                      uint8_t id,
                      uint16_t zero_raw,
                      int8_t angle_sign,
                      int8_t voltage_sign,
                      float gear_ratio)
{
    steering::Gm6020::Config config = MakeConfig();

    config.id = id;
    config.zero_raw = zero_raw;
    config.angle_sign = angle_sign;
    config.voltage_sign = voltage_sign;
    config.gear_ratio = gear_ratio;
    motor.init(hcan, config);
}

static void Test_RecordCall(char call)
{
    if (g_call_order_len < (sizeof(g_call_order) - 1U))
    {
        g_call_order[g_call_order_len++] = call;
        g_call_order[g_call_order_len] = '\0';
    }
}

void Error_Handler(void)
{
    g_failures++;
}

HAL_StatusTypeDef HAL_CAN_ConfigFilter(
    CAN_HandleTypeDef *hcan,
    CAN_FilterTypeDef *filter)
{
    (void)hcan;

    Test_RecordCall('F');

    if (g_filter_count < 4U)
    {
        g_filters[g_filter_count] = *filter;
    }
    g_filter_count++;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    Test_RecordCall('S');
    g_start_count++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_ActivateNotification(
    CAN_HandleTypeDef *hcan,
    uint32_t notification)
{
    (void)hcan;
    (void)notification;
    Test_RecordCall('N');
    g_notify_count++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    CAN_HandleTypeDef *hcan,
    CAN_TxHeaderTypeDef *header,
    uint8_t data[8],
    uint32_t *mailbox)
{
    (void)hcan;

    g_last_tx_header = *header;
    std::memcpy(g_last_tx_data, data, 8U);
    g_tx_count++;

    if (mailbox != NULL)
    {
        *mailbox = 0U;
    }

    return HAL_OK;
}

static void Check(bool condition, const char *message)
{
    if (!condition)
    {
        std::printf("FAIL: %s\n", message);
        g_failures++;
    }
}

static void Test_Wrap180(void)
{
    Check(std::fabs(steering::Gm6020::wrap180(359.0f - 1.0f) + 2.0f) < 0.0001f,
          "Wrap180 359->1");
    Check(std::fabs(steering::Gm6020::wrap180(1.0f - 359.0f) - 2.0f) < 0.0001f,
          "Wrap180 1->359");
    Check(std::fabs(steering::Gm6020::wrap180(180.0f) + 180.0f) < 0.0001f,
          "Wrap180 +180");
    Check(std::fabs(steering::Gm6020::wrap180(-180.0f) + 180.0f) < 0.0001f,
          "Wrap180 -180");
}

static void Test_FeedbackAndTimeout(void)
{
    steering::Gm6020 motor;
    CAN_HandleTypeDef can1 = {CAN1};
    CAN_RxHeaderTypeDef header = {};
    uint8_t data[8] =
    {
        0x0B, 0xE8,
        0x00, 0x64,
        0x00, 0x7B,
        25U, 0U
    };

    InitMotor(motor, &can1, 1U, 1000U, 1, 1, 1.0f);

    header.StdId = 0x205U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;

    motor.onRx(&header, data, 100U);

    Check(motor.raw_angle == 3048U, "raw angle decode");
    Check(std::fabs(motor.angle_deg - 90.0f) < 0.01f,
          "zero/sign angle conversion");
    Check(motor.speed_rpm == 100, "speed decode");
    Check(motor.current_raw == 123, "current decode");
    Check(motor.temperature == 25U, "temperature decode");
    Check(motor.is_online == 1U, "online after feedback");
    Check(std::fabs(motor.target_angle_deg - 90.0f) < 0.01f,
          "first feedback seeds target");

    motor.enable();
    motor.controlStep(120U);
    Check(motor.is_online == 1U, "20 ms is still online");

    motor.controlStep(121U);
    Check(motor.is_online == 0U, "21 ms is offline");
    Check(motor.target_voltage == 0, "timeout clears target voltage");
    Check((g_last_tx_data[0] == 0U) &&
          (g_last_tx_data[1] == 0U),
          "timeout sends zero voltage");
}

static void Test_VoltagePacking(void)
{
    steering::Gm6020 motor;
    CAN_HandleTypeDef can1 = {CAN1};

    InitMotor(motor, &can1, 1U, 0U, 1, 1, 1.0f);
    Check(motor.sendVoltage(1234) == HAL_OK,
          "ID1 send");
    Check(g_last_tx_header.StdId == 0x1FFU,
          "ID1 frame id");
    Check((g_last_tx_data[0] == 0x04U) &&
          (g_last_tx_data[1] == 0xD2U),
          "ID1 first voltage slot");
    Check((g_last_tx_data[2] == 0U) &&
          (g_last_tx_data[3] == 0U) &&
          (g_last_tx_data[4] == 0U) &&
          (g_last_tx_data[5] == 0U) &&
          (g_last_tx_data[6] == 0U) &&
          (g_last_tx_data[7] == 0U),
          "ID1 unused slots zero");
    Check(motor.last_tx_status == HAL_OK,
          "TX status recorded");
    Check(motor.tx_ok_count == 1U,
          "TX success counter");
    Check(motor.last_tx_voltage == 1234,
          "last TX voltage recorded");
    Check((motor.last_tx_data[0] == 0x04U) &&
          (motor.last_tx_data[1] == 0xD2U),
          "last TX data recorded");

    InitMotor(motor, &can1, 5U, 0U, 1, 1, 1.0f);
    Check(motor.sendVoltage(1234) == HAL_OK,
          "ID5 send");
    Check(g_last_tx_header.StdId == 0x2FFU,
          "ID5 frame id");
    Check((g_last_tx_data[0] == 0x04U) &&
          (g_last_tx_data[1] == 0xD2U),
          "ID5 first voltage slot");
    Check(g_last_tx_data[2] == 0U, "ID5 unused slot zero");
}

static void Test_ControlIntegralPolicy(void)
{
    steering::Gm6020 motor;
    CAN_HandleTypeDef can1 = {CAN1};
    float initial_integral;
    int16_t initial_voltage;
    uint32_t i;

    InitMotor(motor, &can1, 1U, 0U, 1, 1, 1.0f);

    Check(motor.position_pid.integral_separation == 1U,
          "position integral separation remains enabled");
    Check(motor.speed_pid.integral_separation == 0U,
          "speed integral separation disabled in driver");

    motor.is_online = 1U;
    motor.is_enabled = 1U;
    motor.voltage_limit_raw = 5000;
    motor.max_speed_rpm = 300.0f;
    motor.target_angle_deg = 15.0f;
    motor.angle_deg = 0.0f;
    motor.speed_rpm = 0;

    for (i = 0U; i < 10U; i++)
    {
        motor.last_rx_ms = i;
        motor.controlStep(i);
    }

    Check(std::fabs(motor.position_pid.output_max - 300.0f) < 0.001f,
          "position anti-windup limit follows max speed");
    Check(std::fabs(motor.speed_pid.output_max - 5000.0f) < 0.001f,
          "speed anti-windup limit follows voltage limit");
    Check(std::fabs(motor.speed_pid.integral) < 0.001f,
          "speed integral does not wind beyond active limit");
    Check(motor.target_voltage == 5000,
          "speed output is limited to active voltage limit");

    InitMotor(motor, &can1, 1U, 0U, 1, 1, 1.0f);
    motor.is_online = 1U;
    motor.is_enabled = 1U;
    motor.target_angle_deg = 4.5f;
    motor.angle_deg = 0.0f;
    motor.speed_rpm = 0;
    motor.position_pid.kd = 0.0f;
    motor.position_pid.integral = 50.0f;
    motor.speed_pid.integral = -4800.0f;
    initial_integral = motor.speed_pid.integral;

    motor.last_rx_ms = 0U;
    motor.controlStep(0U);
    initial_voltage = motor.target_voltage;

    for (i = 1U; i < 100U; i++)
    {
        motor.last_rx_ms = i;
        motor.controlStep(i);
    }

    Check(motor.speed_pid.integral > initial_integral + 500.0f,
          "speed integral recovers from negative bias at standstill");
    Check(motor.target_voltage > initial_voltage + 200,
          "voltage continues increasing at standstill");
}

static void Test_ConfigAndControlModes(void)
{
    steering::Gm6020 motor;
    CAN_HandleTypeDef can1 = {CAN1};
    steering::Gm6020::Config config = MakeConfig();

    config.voltage_sign = -1;
    config.position.kp = 12.0f;
    config.position.ki = 1.0f;
    config.position.kd = 0.2f;
    config.position.integral_max = 40.0f;
    config.position.integral_separation = 1U;
    config.position.separation_threshold = 8.0f;
    config.speed.kp = 60.0f;
    config.speed.ki = 30.0f;
    config.speed.kd = 0.0f;
    config.speed.integral_max = 4000.0f;
    config.speed.integral_separation = 0U;
    config.speed.separation_threshold = 100.0f;
    config.control_dt_s = 0.001f;
    config.max_speed_rpm = 250.0f;
    config.voltage_limit_raw = 5000;
    config.online_timeout_ms = 15U;

    motor.init(&can1, config);

    Check(std::fabs(motor.position_pid.kp - 12.0f) < 0.001f,
          "position config applied during init");
    Check(std::fabs(motor.speed_pid.ki - 30.0f) < 0.001f,
          "speed config applied during init");
    Check(motor.online_timeout_ms == 15U,
          "timeout config applied during init");

    motor.is_online = 1U;
    motor.angle_deg = 0.0f;
    motor.speed_rpm = 0;
    motor.last_rx_ms = 100U;
    motor.enable();
    Check(motor.is_enabled == 1U, "online motor enables");

    motor.setControlMode(steering::Gm6020::ControlMode::Speed);
    Check(motor.is_enabled == 0U,
          "mode change clears enable");
    Check(motor.target_voltage == 0,
          "mode change sends neutral target");
    motor.enable();
    motor.setTargetSpeed(400.0f);
    motor.controlStep(100U);
    Check(std::fabs(motor.target_speed_rpm - 250.0f) < 0.001f,
          "speed target limited to max RPM");
    Check(motor.target_voltage != 0,
          "speed mode drives the speed PID");

    motor.setControlMode(steering::Gm6020::ControlMode::Voltage);
    Check(motor.is_enabled == 0U,
          "voltage mode starts disabled");
    motor.enable();
    motor.setTargetVoltage(8000);
    motor.controlStep(100U);
    Check(motor.target_speed_rpm == 0.0f,
          "voltage mode bypasses speed PID");
    Check(motor.target_voltage == 5000,
          "voltage mode applies runtime limit");
    Check(motor.last_tx_voltage == 5000,
          "voltage mode ignores negative voltage sign");

    motor.setControlMode(steering::Gm6020::ControlMode::Position);
    Check(motor.requested_voltage_raw == 0,
          "mode change clears voltage request");
    Check(motor.is_enabled == 0U,
          "position mode starts disabled");
}

static void Test_ControlModeTimeouts(void)
{
    steering::Gm6020 motor;
    CAN_HandleTypeDef can1 = {CAN1};
    steering::Gm6020::ControlMode modes[] =
    {
        steering::Gm6020::ControlMode::Position,
        steering::Gm6020::ControlMode::Speed,
        steering::Gm6020::ControlMode::Voltage
    };
    uint32_t i;

    for (i = 0U; i < 3U; i++)
    {
        InitMotor(motor, &can1, 1U, 0U, 1, 1, 1.0f);
        motor.is_online = 1U;
        motor.last_rx_ms = 10U;
        motor.angle_deg = 0.0f;
        motor.speed_rpm = 0;
        motor.setControlMode(modes[i]);
        motor.enable();
        motor.setTargetAngle(20.0f);
        motor.setTargetSpeed(50.0f);
        motor.setTargetVoltage(1000);

        motor.controlStep(30U);
        Check(motor.is_online == 1U,
              "mode remains online at timeout boundary");

        motor.controlStep(31U);
        Check(motor.is_online == 0U,
              "mode times out after boundary");
        Check(motor.is_enabled == 0U,
              "timeout clears enable");
        Check(motor.target_voltage == 0,
              "timeout clears voltage");
        Check(motor.requested_speed_rpm == 0.0f,
              "timeout clears speed request");
        Check(motor.requested_voltage_raw == 0,
              "timeout clears voltage request");
        Check((g_last_tx_data[0] == 0U) &&
              (g_last_tx_data[1] == 0U),
              "timeout sends zero output");
    }
}

static void Test_BspCan(void)
{
    CAN_HandleTypeDef can1 = {CAN1};
    CAN_HandleTypeDef can2 = {CAN2};

    g_filter_count = 0U;
    g_start_count = 0U;
    g_notify_count = 0U;
    g_call_order_len = 0U;
    g_call_order[0] = '\0';

    Check(steering::bsp_can::startAcceptAll(&can1) == HAL_OK,
          "accept-all start");
    Check(g_filter_count == 1U, "accept-all filter count");
    Check(g_filters[0].FilterBank == 0U,
          "CAN1 accept-all bank");
    Check((g_filters[0].FilterIdHigh == 0U) &&
          (g_filters[0].FilterMaskIdHigh == 0U) &&
          (g_filters[0].FilterMaskIdLow == 0U),
          "accept-all mask");
    Check(strcmp(g_call_order, "FSN") == 0,
          "filter/start/notify order");

    Check(steering::bsp_can::startMasterFilter(
              &can2, 0x101U, 0x111U) == HAL_OK,
          "master filter start");
    Check(g_filter_count == 3U, "master filter count");
    Check(g_filters[1].FilterBank == 14U,
          "control filter bank");
    Check(g_filters[2].FilterBank == 15U,
          "heartbeat filter bank");
    Check(g_filters[1].FilterIdHigh == (uint32_t)(0x101U << 5U),
          "control standard id");
    Check(g_filters[2].FilterIdHigh == (uint32_t)(0x111U << 5U),
          "heartbeat standard id");
    Check(strcmp(g_call_order, "FSNFFSN") == 0,
          "master filter order");
    Check((g_start_count == 2U) && (g_notify_count == 2U),
          "start and notify called once per bus");
}

static void Test_ConfigValidation(void)
{
    steering::Gm6020 motor;
    CAN_HandleTypeDef can1 = {CAN1};
    steering::Gm6020::Config config = MakeConfig();
    uint32_t failures_before;

    config.control_dt_s = 0.0f;
    failures_before = g_failures;
    motor.init(&can1, config);
    Check(g_failures == failures_before + 1U,
          "zero control period is rejected");
    g_failures = failures_before;

    config = MakeConfig();
    config.voltage_limit_raw =
        static_cast<int32_t>(steering::Gm6020::MaxVoltageRaw) + 1;
    failures_before = g_failures;
    motor.init(&can1, config);
    Check(g_failures == failures_before + 1U,
          "voltage limit above hardware maximum is rejected");
    g_failures = failures_before;
}

int main()
{
    Test_Wrap180();
    Test_FeedbackAndTimeout();
    Test_VoltagePacking();
    Test_ControlIntegralPolicy();
    Test_ConfigAndControlModes();
    Test_ControlModeTimeouts();
    Test_BspCan();
    Test_ConfigValidation();

    if (g_failures != 0U)
    {
        std::printf("GM6020 host tests: %u failure(s)\n",
                    static_cast<unsigned>(g_failures));
        return 1;
    }

    std::printf("GM6020 host tests: all passed\n");
    return 0;
}
