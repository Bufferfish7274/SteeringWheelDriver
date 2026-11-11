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

static void MakeGroupData(uint8_t data[8],
                          int16_t first_erpm,
                          uint16_t first_angle_code,
                          int16_t second_erpm,
                          uint16_t second_angle_code)
{
    const uint16_t first_erpm_raw = (uint16_t)first_erpm;
    const uint16_t second_erpm_raw = (uint16_t)second_erpm;

    data[0] = (uint8_t)(first_erpm_raw >> 8U);
    data[1] = (uint8_t)first_erpm_raw;
    data[2] = (uint8_t)(first_angle_code >> 8U);
    data[3] = (uint8_t)first_angle_code;

    data[4] = (uint8_t)(second_erpm_raw >> 8U);
    data[5] = (uint8_t)second_erpm_raw;
    data[6] = (uint8_t)(second_angle_code >> 8U);
    data[7] = (uint8_t)second_angle_code;
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

    InitMaster(master, 2U);
    Check(master.control_id == 0x101U, "wheel2 uses group1 control id");
    Check(master.heartbeat_id == 0x111U, "wheel2 uses group1 heartbeat id");

    InitMaster(master, 3U);
    Check(master.control_id == 0x102U, "wheel3 uses group2 control id");
    Check(master.heartbeat_id == 0x112U, "wheel3 uses group2 heartbeat id");

    InitMaster(master, 4U);
    Check(master.control_id == 0x102U, "wheel4 uses group2 control id");
    Check(master.heartbeat_id == 0x112U, "wheel4 uses group2 heartbeat id");
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

    MakeGroupData(data, -1234, 8192, 2345, 4096);
    header = MakeHeader(0x101U);
    master.onRx(&header, data, 100U);

    Check(master.drive_erpm == -1234, "wheel1 signed ERPM decode");
    Check(master.angle_code == 8192U, "wheel1 angle code decode");
    Check(std::fabs(master.target_angle_deg - 180.0f) < 0.001f,
          "wheel1 angle code to degrees");
    Check(master.control_seen == 1U, "control available immediately");
    Check(master.link_valid == 1U, "control counts as heartbeat");

    InitMaster(master, 2U);
    master.onRx(&header, data, 105U);
    Check(master.drive_erpm == 2345, "wheel2 uses second ERPM slot");
    Check(master.angle_code == 4096U, "wheel2 uses second angle slot");
    Check(std::fabs(master.target_angle_deg - 90.0f) < 0.001f,
          "wheel2 angle code to degrees");

    InitMaster(master, 3U);
    header = MakeHeader(0x102U);
    master.onRx(&header, data, 110U);
    Check(master.drive_erpm == -1234, "wheel3 uses first ERPM slot");
    Check(master.angle_code == 8192U, "wheel3 uses first angle slot");

    InitMaster(master, 4U);
    master.onRx(&header, data, 115U);
    Check(master.drive_erpm == 2345, "wheel4 uses second ERPM slot");
    Check(master.angle_code == 4096U, "wheel4 uses second angle slot");

    InitMaster(master, 1U);
    MakeGroupData(data, INT16_MIN, 0U, INT16_MAX, 16383U);
    header = MakeHeader(0x101U);
    master.onRx(&header, data, 120U);
    Check(master.drive_erpm == INT16_MIN, "minimum int16 ERPM decode");
    Check(master.angle_code == 0U, "zero angle code");

    InitMaster(master, 2U);
    master.onRx(&header, data, 125U);
    Check(master.drive_erpm == INT16_MAX, "maximum int16 ERPM decode");
    Check(master.angle_code == 16383U, "angle upper bound");

    MakeGroupData(data, 0, 16384U, 0, 0U);
    master.onRx(&header, data, 130U);
    Check(master.angle_code == 0U, "wheel2 ignores wheel1 angle");

    InitMaster(master, 1U);
    master.onRx(&header, data, 110U);
    Check(master.angle_code == 16383U, "angle over upper bound clamps");

    MakeGroupData(data, 0, 0xFFFFU, 0, 0U);
    master.onRx(&header, data, 140U);
    Check(master.angle_code == 16383U, "0xFFFF angle clamps");
}

static void Test_InputValidation(void)
{
    steering::MasterCan master;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8] = {0};

    InitMaster(master, 1U);

    header = MakeHeader(0x102U);
    master.onRx(&header, data, 100U);
    Check(master.control_seen == 0U, "other group control id ignored");
    Check(master.link_valid == 0U, "other group control does not refresh link");

    header = MakeHeader(0x112U);
    master.onRx(&header, data, 105U);
    Check(master.link_valid == 0U, "other group heartbeat ignored");

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
    MakeGroupData(data, 0, 0U, 100, 0U);
    header = MakeHeader(0x101U);
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
    header = MakeHeader(0x111U);
    master.onRx(&header, data, 100U);
    Check(master.link_valid == 1U, "heartbeat alone keeps link");
    Check(master.control_seen == 0U, "heartbeat does not create control");

    MakeGroupData(data, 0, 0U, 100, 0U);
    header = MakeHeader(0x101U);
    master.onRx(&header, data, 100U);
    header = MakeHeader(0x111U);
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

    header = MakeHeader(0x111U);
    master.onRx(&header, data, 1000U);
    Check(master.link_valid == 1U, "heartbeat restores link");
    Check(master.control_seen == 0U,
          "heartbeat recovery does not restore old control");

    MakeGroupData(data, 999, 8192U, 100, 0U);
    header = MakeHeader(0x101U);
    master.onRx(&header, data, 1100U);
    Check(master.control_seen == 1U,
          "group frame restores available command");
    Check(master.drive_erpm == 100,
          "peer target change preserves this wheel target");
    Check(master.angle_code == 0U,
          "peer angle change preserves this wheel angle");

    MakeGroupData(data, 999, 8192U, 200, 4096U);
    master.onRx(&header, data, 1200U);
    Check(master.drive_erpm == 200, "new group target replaces old target");
    Check(master.angle_code == 4096U,
          "new group angle replaces old angle");
}

static void Test_TimeWrap(void)
{
    steering::MasterCan master;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    InitMaster(master, 1U);
    MakeGroupData(data, 0, 0U, 0, 0U);
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
