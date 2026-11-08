#include <cmath>
#include <cstdint>
#include <cstdio>

#include "MasterCan.hpp"

static uint32_t g_failures;

static void InitMaster(steering::MasterCan &master, uint32_t wheel_id)
{
    steering::MasterCan::Config config = {};
    config.wheel_id = wheel_id;
    config.link_timeout_ms = 500U;

    master.init(config);
}

static void Check(bool condition, const char *message)
{
    if (!condition)
    {
        std::printf("FAIL: %s\n", message);
        g_failures++;
    }
}

void Error_Handler(void)
{
    g_failures++;
}

static void MakeControlData(uint8_t data[8],
                            int32_t erpm,
                            int32_t angle_code)
{
    uint32_t erpm_raw = (uint32_t)erpm;
    uint32_t angle_raw = (uint32_t)angle_code;

    data[0] = (uint8_t)(erpm_raw >> 24U);
    data[1] = (uint8_t)(erpm_raw >> 16U);
    data[2] = (uint8_t)(erpm_raw >> 8U);
    data[3] = (uint8_t)erpm_raw;

    data[4] = (uint8_t)(angle_raw >> 24U);
    data[5] = (uint8_t)(angle_raw >> 16U);
    data[6] = (uint8_t)(angle_raw >> 8U);
    data[7] = (uint8_t)angle_raw;
}

static CAN_RxHeaderTypeDef MakeHeader(uint32_t std_id)
{
    CAN_RxHeaderTypeDef header = {};

    header.StdId = std_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    return header;
}

static void Test_WheelIds(void)
{
    steering::MasterCan master;

    InitMaster(master, 1U);
    Check(master.control_id == 0x101U, "wheel1 control id");
    Check(master.heartbeat_id == 0x111U, "wheel1 heartbeat id");

    InitMaster(master, 4U);
    Check(master.control_id == 0x104U, "wheel4 control id");
    Check(master.heartbeat_id == 0x114U, "wheel4 heartbeat id");
}

static void Test_Decode(void)
{
    steering::MasterCan master;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    InitMaster(master, 1U);
    master.update(0U);
    Check(master.control_seen == 0U, "no control before first frame");
    Check(master.link_valid == 0U, "no link before first frame");

    MakeControlData(data, -1234, 8192);
    header = MakeHeader(0x101U);
    master.onRx(&header, data, 100U);

    Check(master.drive_erpm == -1234, "signed ERPM decode");
    Check(master.angle_code == 8192U, "angle code decode");
    Check(std::fabs(master.target_angle_deg - 180.0f) < 0.001f,
          "angle code to degrees");
    Check(master.control_seen == 1U, "control available immediately");
    Check(master.link_valid == 1U, "control counts as heartbeat");

    MakeControlData(data, 0, 16383);
    master.onRx(&header, data, 110U);
    Check(master.angle_code == 16383U, "angle upper bound");

    MakeControlData(data, 0, 16384);
    master.onRx(&header, data, 120U);
    Check(master.angle_code == 16383U, "angle over upper bound clamps");

    MakeControlData(data, 0, -1);
    master.onRx(&header, data, 130U);
    Check(master.angle_code == 0U, "negative angle clamps to zero");
}

static void Test_InputValidation(void)
{
    steering::MasterCan master;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8] = {0};

    InitMaster(master, 1U);

    header = MakeHeader(0x102U);
    master.onRx(&header, data, 100U);
    Check(master.control_seen == 0U, "wrong control id ignored");

    header = MakeHeader(0x101U);
    header.IDE = CAN_ID_EXT;
    master.onRx(&header, data, 110U);
    Check(master.control_seen == 0U, "extended frame ignored");

    header = MakeHeader(0x101U);
    header.RTR = 1U;
    master.onRx(&header, data, 120U);
    Check(master.control_seen == 0U, "remote frame ignored");

    header = MakeHeader(0x101U);
    header.DLC = 7U;
    master.onRx(&header, data, 130U);
    Check(master.control_seen == 0U, "wrong DLC ignored");
}

static void Test_TimeoutAndHeartbeat(void)
{
    steering::MasterCan master;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    InitMaster(master, 2U);
    MakeControlData(data, 100, 0);
    header = MakeHeader(0x102U);
    master.onRx(&header, data, 100U);

    master.update(300U);
    Check(master.control_seen == 1U,
          "control persists without periodic updates");
    Check(master.link_valid == 1U, "control maintains link");

    master.update(600U);
    Check(master.control_seen == 1U, "control valid at link boundary");
    Check(master.link_valid == 1U, "500 ms link still valid");
    master.update(601U);
    Check(master.link_valid == 0U, "501 ms link invalid");
    Check(master.control_seen == 0U,
          "link expiry clears available control");
    Check(master.drive_erpm == 100,
          "link expiry preserves last decoded ERPM");

    InitMaster(master, 2U);
    header = MakeHeader(0x112U);
    master.onRx(&header, data, 100U);
    Check(master.link_valid == 1U, "heartbeat alone keeps link");
    Check(master.control_seen == 0U, "heartbeat does not create control");

    MakeControlData(data, 100, 0);
    header = MakeHeader(0x102U);
    master.onRx(&header, data, 100U);
    header = MakeHeader(0x112U);
    master.onRx(&header, data, 400U);
    master.update(500U);
    Check(master.control_seen == 1U,
          "heartbeat keeps persistent control available");
    Check(master.link_valid == 1U, "heartbeat refreshes link");

    master.update(900U);
    Check(master.link_valid == 1U,
          "heartbeat valid at 500 ms boundary");

    master.update(901U);
    Check(master.control_seen == 0U,
          "timeout clears control before heartbeat recovery");
    Check(master.link_valid == 0U,
          "heartbeat link expires after 500 ms");

    header = MakeHeader(0x112U);
    master.onRx(&header, data, 1000U);
    Check(master.link_valid == 1U, "heartbeat restores link");
    Check(master.control_seen == 0U,
          "heartbeat recovery does not restore old control");

    MakeControlData(data, 200, 4096);
    header = MakeHeader(0x102U);
    master.onRx(&header, data, 1100U);
    Check(master.control_seen == 1U,
          "new control restores available command");
    Check(master.drive_erpm == 200, "new control replaces old target");
}

static void Test_TimeWrap(void)
{
    steering::MasterCan master;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    InitMaster(master, 1U);
    MakeControlData(data, 0, 0);
    header = MakeHeader(0x101U);
    master.onRx(&header, data, 0xFFFFFFF0U);

    master.update(0x00000010U);
    Check(master.control_seen == 1U, "time wrap control available");
    master.update(0x000000F0U);
    Check(master.control_seen == 1U, "time wrap control persists");
    Check(master.link_valid == 1U, "time wrap link valid");
    master.update(0x000001F0U);
    Check(master.link_valid == 0U, "time wrap link expired");
    Check(master.control_seen == 0U, "time wrap clears control");
}

static void Test_ConfigValidation(void)
{
    steering::MasterCan master;
    steering::MasterCan::Config config = {};
    config.wheel_id = 5U;
    config.link_timeout_ms = 500U;
    uint32_t failures_before = g_failures;

    master.init(config);
    Check(g_failures == failures_before + 1U,
          "wheel ID above protocol range is rejected");
    g_failures = failures_before;
}

int main()
{
    Test_WheelIds();
    Test_Decode();
    Test_InputValidation();
    Test_TimeoutAndHeartbeat();
    Test_TimeWrap();
    Test_ConfigValidation();

    if (g_failures != 0U)
    {
        std::printf("MasterCan host tests: %u failure(s)\n",
                    static_cast<unsigned>(g_failures));
        return 1;
    }

    std::printf("MasterCan host tests: all passed\n");
    return 0;
}
