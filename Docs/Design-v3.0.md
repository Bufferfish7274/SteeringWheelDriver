# 舵轮分控板驱动 3.0 设计说明

## 文档信息

| 项目 | 内容 |
| --- | --- |
| 版本 | 3.0 |
| 日期 | 2026-11-06 |
| 控制器 | STM32F405RGT6 |
| 编译器 | Arm Compiler 6.22 |
| 语言 | C++17（手写层）+ GNU C11（HAL/CubeMX） |
| 适用范围 | 单舵轮、单 GM6020、单 VESC |
| 配套文档 | [项目 README](../README.md)、[3.0 测试说明](../Tests/README.md)、[调试模式](DebugMode.md)、[模块文档](../Lib/Docs/README.md) |

3.0 将 2.0 的类型化配置和函数式分层重组为 C++17 核心，同时保持 CAN协议、安全逻辑、串口列序和 Keil Watch 调参路径不变。

## 1. 设计目标

实际上，2.0版本的代码组织风格已经接近C++。因此，3.0 的目标不是引入运行时多态或复杂 C++ 框架，而是让对象所有权、生命周期、配置注入和模块边界在编译期明确：

1. 手写 App、驱动和算法全部使用 C++17；
2. CubeMX `Core`、启动文件和 STM32 HAL 保持 C11；
3. C 与 C++ 只通过 `WheelAppApi.h` 的 C ABI 交互；
4. 不使用堆、STL 容器、异常、RTTI 类型识别或虚函数；
5. 正式固件与调试固件继续通过 `WHEEL_BUILD_DEBUG` 单 Target 切换；
6. Watch 保留 `g_app.gm6020.*`、`g_app.vesc.*` 和 `g_wheel_debug.*`。

## 2. 系统结构

```text
Core/Src/main.c（C）
  │ HAL 初始化、中断入口、C ABI 调用
  ▼
WheelAppApi.h / WheelAppApi.cpp（extern "C" 薄层）
  ▼
WheelApp（C++ 生命周期与共享安全状态）
  ├─ ProductionController  正式控制策略
  ├─ DebugController       调试控制策略
  ├─ Gm6020
  ├─ Vesc
  ├─ MasterCan
  ├─ SetpointRamp
  ├─ WheelDebug
  ├─ Telemetry
  └─ StatusLeds
           ▼
BspCan → STM32 HAL（C）
```

`main.c` 不再持有 `App_t`。全局单例 `g_app` 位于 C++ 实现中，所有 HAL
回调和主循环均通过 `WheelApp_*` C ABI 调用。

## 3. 模块职责

| 模块                     | C++ 形态       | 主要职责                            |
| ---------------------- | ------------ | ------------------------------- |
| `WheelApp`             | 类            | 对象所有权、HAL 事件、1 kHz 调度、公共在线和安全状态 |
| `ProductionController` | 类            | 主控新鲜命令、坐标映射、AngleOpt、整轮安全与目标下发  |
| `DebugController`      | 类            | Watch 模式、阶跃/正弦、调试使能和设备在线门控      |
| `Gm6020`               | 类            | 反馈解码、两种 PID、三种控制模式和 CAN 发送      |
| `Vesc`                 | 类            | ERPM 指令、状态反馈、在线超时和控制模式          |
| `MasterCan`            | 类            | 主控控制/心跳解析、目标持久和链路恢复             |
| `Pid`                  | 类            | 积分分离、抗饱和和位置式 PID                |
| `SetpointRamp`         | 类            | 位置参考速度/加速度限制                    |
| `WheelDebug`           | 类 + Watch 结构 | 调试输入和波形目标生成                     |
| `AngleOpt`             | 命名空间函数       | 无状态物理等价舵向选择                     |
| `BspCan`               | 命名空间函数       | 无状态 bxCAN 过滤器和启动配置              |
| `Telemetry`            | 类            | 只读 CSV 组装和非阻塞 UART 发送           |
| `StatusLeds`           | 类            | 只读状态到 LED 的映射                   |

正式和调试控制器在编译期二选一。二者使用 `WheelApp` 中的同一批驱动、
控制周期和位置斜坡对象，不创建调试状态副本。

## 4. 编译配置

Keil Target 使用与战队工程一致的 AC6 配置：

- C：GNU11；
- C++：C++17；
- Optimization：`-O3`；
- One ELF Section per Function；
- AC5-like Warnings；
- Short enums / `short wchar`；
- Misc Controls：`-frtti`；
- 不启用异常（AC6 生成 `-fno-exceptions`）；
- 宏：`USE_HAL_DRIVER,STM32F405xx`；
- 包含路径：`Core/Inc`、HAL/CMSIS、`Lib`、`App`。

源代码约定：

- 手写实现使用 `.cpp`，接口使用 `.hpp`；
- CubeMX/HAL 继续使用 `.c/.h`；
- 不在中断路径使用动态分配、阻塞调用或异常；
- 硬件对象禁止复制，配置在 `init()` 中单次注入；
- 全局运行对象依赖静态清零和显式 `init/reset()`，不依赖动态初始化。

## 5. 配置与 Watch

`AppConfig.hpp` 是唯一生产参数来源，参数使用
`steering::config::inline constexpr` 保存。`WheelApp` 在初始化时组装
`Gm6020::Config`、`Vesc::Config`、`MasterCan::Config`、
`SetpointRampConfig` 和 `WheelDebugConfig`，一次传给对应模块。

驱动不包含 `AppConfig.hpp`，也不保留产品默认 PID、斜坡或超时。配置
在启动外设前校验，非法值直接进入 `Error_Handler()`。

为保留现有台架流程，以下 Watch 路径保持有效：

```text
g_app.gm6020.position_pid.kp
g_app.gm6020.position_pid.ki
g_app.gm6020.position_pid.kd
g_app.gm6020.speed_pid.kp
g_app.gm6020.speed_pid.ki
g_app.gm6020.speed_pid.kd
g_app.gm6020.max_speed_rpm
g_app.gm6020.voltage_limit_raw
g_app.gm6020.angle_deg
g_app.gm6020.speed_rpm
g_app.vesc.actual_erpm
g_wheel_debug.mode
g_wheel_debug.enable
```

`g_app` 与 `g_wheel_debug` 均位于 `.bss`，Watch 修改的是实际运行对象，
不是配置副本。修改不会写入 Flash，停机后仍需回填 `AppConfig.hpp`。

## 6. 运行时协调

| 事件 | 周期 | 执行内容 |
| --- | --- | --- |
| CAN RX | 帧到达 | 正式：主控解码；调试：反馈解码 |
| TIM6 | 1 kHz | 在线判断、安全门控、控制器策略、PID 和 CAN 发送 |
| LED | 10 ms | 在线、使能和调试心跳 |
| UART | 20 ms | 50 Hz 非阻塞遥测 |
| UART TX complete | 中断 | 释放下一次发送许可 |

正式固件采用整轮安全条件：主控链路有效、已有控制目标、GM6020 在线、
VESC 在线。任一条件失效时两个执行器失能，并清除待执行的新命令标志。
恢复后必须收到新的主控控制帧。

## 7. 测试

全部主机测试使用：

```text
g++ -std=c++17 -Wall -Wextra -Werror -fno-exceptions -fno-rtti
```

### 7.1 主机测试矩阵

| 测试 | 覆盖内容 |
| --- | --- |
| `Tests/App` | 调试与正式两个构建、CAN1/CAN2 启动、配置注入、整轮使能和断线停机 |
| `Tests/GM6020` | 角度换算、反馈超时、PID、三种模式、CAN 槽位和非法配置 |
| `Tests/VESC` | ERPM/电流/占空比解码、超时、CAN 发送和非法 ID |
| `Tests/MasterCan` | 控制/心跳 ID、角度边界、持久目标、恢复和时间回绕 |
| `Tests/WheelDebug` | Watch 模式、阶跃、正弦、180° 等价候选和斜坡共享 |
| `Tests/AngleOpt` | 最近物理等价方向、90° 最大选择位移和边界 |
| `Tests/SetpointRamp` | 速度/加速度限制、跨零路径和运行中目标变化 |

在项目根目录运行命令：

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\App\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\GM6020\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\VESC\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\MasterCan\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\WheelDebug\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\AngleOpt\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\SetpointRamp\host\run.ps1
```

### 7.2 台架验收

主机测试不能替代真实电机验证。烧录前必须固定机构，并按调试文档完成：

1. `DISABLED` 模式读取原始角并确认零位；
2. 小角度位置阶跃确认角度方向和 `angle_sign`；
3. 小电压测试确认 `voltage_sign` 和功率链路；
4. 先速度环、后位置环调参；
5. VESC 小 ERPM 测试；
6. 正式固件联调主控链路、断线失能和恢复后新命令要求。
