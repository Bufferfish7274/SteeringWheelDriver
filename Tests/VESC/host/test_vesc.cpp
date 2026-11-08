#include <cstdio>
#include <cstring>

#include "VESC.hpp"

const int VESC_STUB_CAN1 = 1;

static CAN_TxHeaderTypeDef g_tx_header;
static uint8_t g_tx_data[8];
static uint32_t g_tx_count;
static HAL_StatusTypeDef g_tx_result = HAL_OK;
static uint32_t g_failures;

static void InitVesc(steering::Vesc &vesc, CAN_HandleTypeDef *hcan)
{
    steering::Vesc::Config config = {};
    config.controller_id = 67U;
    config.online_timeout_ms = 100U;

    vesc.init(hcan, config);
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

    g_tx_header = *header;
    std::memcpy(g_tx_data, data, sizeof(g_tx_data));
    g_tx_count++;

    if (mailbox != nullptr)
    {
        *mailbox = 0U;
    }

    return g_tx_result;
}

static void Check(bool condition, const char *message)
{
    if (!condition)
    {
        std::printf("FAIL: %s\n", message);
        g_failures++;
    }
}

static void EncodeInt32(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)(raw >> 24U);
    data[1] = (uint8_t)(raw >> 16U);
    data[2] = (uint8_t)(raw >> 8U);
    data[3] = (uint8_t)raw;
}

static void EncodeInt16(uint8_t *data, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    data[0] = (uint8_t)(raw >> 8U);
    data[1] = (uint8_t)raw;
}

static void MakeStatusHeader(CAN_RxHeaderTypeDef *header)
{
    std::memset(header, 0, sizeof(*header));
    header->ExtId =
        (steering::Vesc::kPacketStatus << 8U) | 67U;
    header->IDE = CAN_ID_EXT;
    header->RTR = CAN_RTR_DATA;
    header->DLC = 8U;
}

static void Test_StatusDecode(void)
{
    steering::Vesc vesc;
    CAN_HandleTypeDef can1 = {&VESC_STUB_CAN1};
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    InitVesc(vesc, &can1);
    MakeStatusHeader(&header);
    EncodeInt32(&data[0], -123456);
    EncodeInt16(&data[4], -123);
    EncodeInt16(&data[6], 750);

    vesc.onRx(&header, data, 1000U);

    Check(vesc.actual_erpm == -123456, "signed ERPM decode");
    Check(vesc.actual_current_a < -12.29f &&
          vesc.actual_current_a > -12.31f,
          "current decode");
    Check(vesc.duty > 0.749f && vesc.duty < 0.751f,
          "duty decode");
    Check(vesc.is_online == 1U, "valid status marks online");
    Check(vesc.last_rx_ms == 1000U, "status timestamp");

    header.ExtId =
        (steering::Vesc::kPacketStatus << 8U) | 68U;
    vesc.onRx(&header, data, 1010U);
    Check(vesc.last_rx_ms == 1000U, "wrong controller ID ignored");

    MakeStatusHeader(&header);
    header.IDE = CAN_ID_STD;
    vesc.onRx(&header, data, 1020U);
    Check(vesc.last_rx_ms == 1000U, "standard frame ignored");

    MakeStatusHeader(&header);
    header.DLC = 7U;
    vesc.onRx(&header, data, 1030U);
    Check(vesc.last_rx_ms == 1000U, "wrong DLC ignored");

    MakeStatusHeader(&header);
    header.RTR = CAN_RTR_REMOTE;
    vesc.onRx(&header, data, 1040U);
    Check(vesc.last_rx_ms == 1000U, "remote frame ignored");
}

static void Test_TimeoutBoundary(void)
{
    steering::Vesc vesc;
    CAN_HandleTypeDef can1 = {&VESC_STUB_CAN1};
    CAN_RxHeaderTypeDef header;
    uint8_t data[8] = {0};

    InitVesc(vesc, &can1);
    MakeStatusHeader(&header);
    vesc.onRx(&header, data, 0U);
    vesc.enable();
    vesc.setTargetErpm(-321);

    vesc.controlStep(100U);
    Check(vesc.is_online == 1U, "100 ms remains online");
    Check(g_tx_data[0] == 0xFFU && g_tx_data[1] == 0xFFU &&
          g_tx_data[2] == 0xFEU && g_tx_data[3] == 0xBFU,
          "enabled target sent via SET_RPM");

    vesc.controlStep(101U);
    Check(vesc.is_online == 0U, "101 ms marks offline");
    Check(g_tx_data[0] == 0U && g_tx_data[1] == 0U &&
          g_tx_data[2] == 0U && g_tx_data[3] == 0U,
          "offline sends zero ERPM");
}

static void Test_TxPacket(void)
{
    steering::Vesc vesc;
    CAN_HandleTypeDef can1 = {&VESC_STUB_CAN1};
    CAN_RxHeaderTypeDef status;
    uint8_t data[8] = {0};
    uint32_t before;

    InitVesc(vesc, &can1);
    MakeStatusHeader(&status);
    vesc.onRx(&status, data, 1U);
    vesc.enable();
    vesc.setTargetErpm(1234);

    before = g_tx_count;
    Check(vesc.sendErpm(1234) == HAL_OK,
          "SET_RPM send succeeds");
    Check(g_tx_count == before + 1U, "one CAN frame transmitted");
    Check(g_tx_header.ExtId == 0x343U, "ID 67 SET_RPM extended ID");
    Check(g_tx_header.StdId == 0U, "extended frame clears StdId");
    Check(g_tx_header.IDE == CAN_ID_EXT, "SET_RPM uses extended frame");
    Check(g_tx_header.DLC == 8U, "SET_RPM DLC is 8");
    Check(g_tx_data[0] == 0U && g_tx_data[1] == 0U &&
          g_tx_data[2] == 0x04U && g_tx_data[3] == 0xD2U,
          "SET_RPM big-endian ERPM");
    Check(g_tx_data[4] == 0U && g_tx_data[5] == 0U &&
          g_tx_data[6] == 0U && g_tx_data[7] == 0U,
          "SET_RPM tail bytes zero");
    Check(vesc.last_tx_id == 0x343U, "last TX ID recorded");
    Check(vesc.last_tx_erpm == 1234, "last TX ERPM recorded");
}

static void Test_DisableAndCounters(void)
{
    steering::Vesc vesc;
    CAN_HandleTypeDef can1 = {&VESC_STUB_CAN1};
    uint32_t before;

    InitVesc(vesc, &can1);
    g_tx_result = HAL_BUSY;
    before = vesc.tx_busy_count;
    Check(vesc.sendErpm(100) == HAL_BUSY,
          "busy status returned");
    Check(vesc.tx_busy_count == before + 1U,
          "busy counter increments");

    g_tx_result = HAL_OK;
    vesc.enable();
    vesc.setTargetErpm(500);
    vesc.disable();
    Check(vesc.is_enabled == 0U, "disable clears enabled");
    Check(vesc.target_erpm == 0, "disable clears target");
    Check(g_tx_data[0] == 0U && g_tx_data[1] == 0U &&
          g_tx_data[2] == 0U && g_tx_data[3] == 0U,
          "disable sends zero ERPM");
}

static void Test_ConfigValidation(void)
{
    steering::Vesc vesc;
    CAN_HandleTypeDef can1 = {&VESC_STUB_CAN1};
    steering::Vesc::Config config = {};
    config.controller_id = 256U;
    config.online_timeout_ms = 100U;
    uint32_t failures_before = g_failures;

    vesc.init(&can1, config);
    Check(g_failures == failures_before + 1U,
          "controller ID above 255 is rejected");
    g_failures = failures_before;
}

int main()
{
    Test_StatusDecode();
    Test_TimeoutBoundary();
    Test_TxPacket();
    Test_DisableAndCounters();
    Test_ConfigValidation();

    if (g_failures != 0U)
    {
        std::printf("VESC host tests: %u failure(s)\n",
                    static_cast<unsigned>(g_failures));
        return 1;
    }

    std::printf("VESC host tests: all passed\n");
    return 0;
}
