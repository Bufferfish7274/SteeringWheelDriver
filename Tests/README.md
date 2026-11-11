# 舵轮分控板驱动 3.0 测试说明

## 测试目标

3.0 测试验证 C++17 重构没有改变 2.0 的外部行为，并覆盖新增的
C/C++ ABI 边界、配置单次注入和正式/调试控制器编译期切换。

测试不替代真实 CAN、电机、功率桥和机械台架验收。

## 工具链

- 目标机构建：Arm Compiler 6.22；
- C 语言：GNU11；
- C++ 语言：C++17；
- 主机编译器：`g++ -std=c++17`；
- 主机告警：`-Wall -Wextra -Werror`；
- 主机禁用异常和 RTTI：`-fno-exceptions -fno-rtti`。

## 主机测试矩阵

| 目录 | 主要覆盖 |
| --- | --- |
| `Tests/App` | 调试与正式构建、CAN1/CAN2、配置注入、整轮安全、超时恢复 |
| `Tests/GM6020` | 角度换算、Pid、三种控制模式、CAN 打包、配置校验 |
| `Tests/VESC` | 状态解码、ERPM 发送、在线超时和非法配置 |
| `Tests/MasterCan` | 控制/心跳解析、持久目标、链路超时和时间回绕 |
| `Tests/WheelDebug` | Watch 模式、阶跃/正弦、AngleOpt 和共享斜坡 |
| `Tests/AngleOpt` | 最近物理等价方向和最大 90° 选择位移 |
| `Tests/SetpointRamp` | 速度/加速度限制、跨零路径和目标切换 |

## 运行全部主机测试

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\App\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\GM6020\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\VESC\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\MasterCan\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\WheelDebug\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\AngleOpt\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\SetpointRamp\host\run.ps1
```

`Tests/App/host/run.ps1` 会分别构建并运行
`WHEEL_BUILD_DEBUG=1` 与 `WHEEL_BUILD_DEBUG=0`。

## 目标机验收

使用：

```powershell
D:\keil5\UV4\UV4.exe -b MDK-ARM\SteeringWheelDriver.uvprojx -j0 -o build.log
```

验收条件：

- 0 Error、0 Warning；
- C/C++ 文件分别使用 GNU11/C++17；
- `g_app` 和 `g_wheel_debug` 位于 `.bss`；
- 正式版使用 `0x101`/`0x102` 组控制、`0x111`/`0x112` 组心跳、
  DLC 8 和每轮 4 字节槽位；
- 正式版 27 列遥测不变；
- 调试版前 27 列及附加列顺序不变；
- 无堆分配和非必要动态初始化；
- RAM 总量不超过 2.0 AC5 基线的合理范围。

## 台架测试

完整调零、方向检查、GM6020 调参、VESC 测试和正式联调步骤见
[调试模式](../Docs/DebugMode.md)。3.0 架构和测试边界见
[3.0 设计说明](../Docs/Design-v3.0.md)。

## 本次验证结果

验证日期：2026-11-11。

| 项目 | 结果 |
| --- | --- |
| 7 组主机测试 | 全部通过，包括正式/调试 App 双构建 |
| 正式固件 | 0 Error，0 Warning；Code 16608，RO 640，RW 12，ZI 2796 |
| 调试固件 | 0 Error，0 Warning；Code 18492，RO 708，RW 12，ZI 2836 |
| 最大静态调用栈 | 368 bytes |
| Watch 对象 | `g_app` 和 `g_wheel_debug` 位于 `.bss` |
| 2.0 AC5 参照 | RO 21536 bytes，RAM 2832 bytes |
