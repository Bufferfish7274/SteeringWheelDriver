# VESC 模块测试

本目录包含 VESC CAN 驱动的主机端测试。实机速度测试使用调试固件的
`VESC_SPEED` 模式。

## VESC 设置

板级测试按以下 VESC Tool 配置编写：

| 项目 | 设置 |
|---|---|
| VESC ID | 67 |
| Timeout | 100 ms |
| Timeout Brake Current | 0.00 A |
| Can Status Message Mode | CAN_STATUS_1_2_3_4_5 |
| Can Status Rate | 50 Hz |
| CAN Baud Rate | 1 Mbps |
| CAN Mode | VESC |
| APP to Use | No App |

驱动使用扩展帧：

- 控制：`(VESC_PACKET_SET_RPM << 8) | 67 = 0x343`；
- 状态：`(VESC_PACKET_STATUS << 8) | 67 = 0x943`；
- 控制数据为前 4 字节大端 `int32 ERPM`，后 4 字节为 0；
- 状态数据为 ERPM、电流和占空比。

VESC 的命令单位是电气转速 ERPM，不是电机机械 RPM。

## 硬件接线

| 信号 | 连接 |
|---|---|
| VESC CAN_H/CAN_L | 分控板 CAN1_H/CAN1_L |
| VESC GND | 分控板 GND |
| ST-Link SWDIO/SWCLK/GND | 分控板对应 SWD 引脚 |
| USB-TTL RX | PA9（USART1_TX） |
| USB-TTL GND | 分控板 GND |

CAN 总线两端各接 `120 Ω` 终端电阻。首次测试必须可靠固定电机和舵轮，避免轮子自由甩动。

## 测试模式

把 `App/AppConfig.h` 的 `WHEEL_BUILD_DEBUG` 改为 1，重新编译后在
Keil Watch 中设置：

```text
g_wheel_debug.mode = 5
g_wheel_debug.target_erpm = 100
g_wheel_debug.erpm_limit = 1000
g_wheel_debug.enable = 1
```

GM6020 在该模式下保持失能零输出。

## 安全行为

- 初始模式为 `DISABLED`，初始使能为 0；
- 模式切换会立即发送 0 ERPM 并清除使能；
- 超过 `APP_VESC_ONLINE_TIMEOUT_MS` 未收到 ID 67 的有效状态帧时，App
  清除调试使能、发送 0 ERPM，并要求重新手动使能；
- VESC 自身同时配置了 100 ms 通讯超时；
- VESC 离线时 PB5 熄灭，驱动使能时 PB6 点亮；
- 默认 `1000 ERPM` 是首次台架安全上限，不代表电机最终允许转速，确认方向和安全空间后再调整。

LED 均为低电平点亮：

- PB4：1 Hz 调试心跳；
- PB5：VESC 状态在线；
- PB6：VESC 驱动已使能；
- PB7：当前预留，保持熄灭。

## UART 遥测

USART1 保持前 27 列正式遥测格式，调试固件在末尾追加调试模式和 VESC
ERPM 上限。目标 ERPM、实际 ERPM、电流、占空比和在线状态仍在前 27 列中。

## 主机端测试

运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\VESC\host\run.ps1
```

覆盖内容包括：

- ID 67 的状态帧扩展 ID；
- 有符号 ERPM、电流和占空比解码；
- 错误控制器 ID、标准帧和错误周期帧过滤；
- 配置的 100 ms/101 ms 超时边界；
- 离线或失能时发送 0 ERPM；
- `SET_RPM` 的扩展 ID、DLC、大端数据和尾部零字节；
- TX 忙计数和失能清零。

## 板级测试步骤

1. 固定电机，只连接 CAN 和必要的低压控制电源。
2. 烧录后确认四个 LED 自检，随后 PB4 心跳。
3. 确认 VESC 上电且 ID 为 67，PB5 应在状态帧到达后点亮。
4. 保持 `g_wheel_debug.mode=0`、`enable=0`，确认 UART 中 VESC 在线为 1。
5. 设置 `g_wheel_debug.mode=5`，`target_erpm=0`，再设置 `enable=1`。
6. 用小目标做阶跃，例如 `+100 → 0 → -100 → 0 ERPM`，确认转向和反馈符号正确。
7. 逐步增大阶跃，始终不超过 `g_wheel_debug.erpm_limit`。
8. 在 Vofa 中观察目标/实际 ERPM、上升时间、超调和稳态误差。
9. 拔掉 VESC CAN，确认 PB5/PB6 熄灭，软件发送 0 ERPM，并要求重新使能。
10. 恢复 CAN，确认只恢复状态显示，电机不会自动重新起转。

## 参数记录

每次阶跃测试记录：

| 日期 | 目标 ERPM | 上限 ERPM | 上升时间 | 超调 | 稳态误差 | 最大电流 | 备注 |
|---|---:|---:|---:|---:|---:|---:|---|
|  |  |  |  |  |  |  |  |
