# 舵轮分控板驱动 2.0 设计说明

## 文档信息

| 项目   | 内容                              |
| ---- | ------------------------------- |
| 版本   | 3.0                             |
| 日期   | 2026-11-06                      |
| 控制器  | STM32F405RGT6                   |
| 编译器  | Arm Compiler 6.22               |
| 语言   | C++17（手写层）+ GNU C11（HAL/CubeMX） |
| 适用范围 | 单舵轮、单 GM6020、单 VESC             |

3.0 将 2.0 的类型化配置和函数式分层重组为 C++17 核心，同时保持其他部分逻辑不变。

# 版本构建思路

实际上，2.0版本在思路上已经十分接近C++：每个模块有自己的结构体，结构体内包含了所有所需的变量。而且模块内部的变量和函数实际上有属于自己的访问范围。而且存在“结构体嵌套结构体”的现象。受制于C的语法，有些部分的封装只能是差强人意的。因此把驱动改为C++，每个模块为一个类，利用继承、多态、访问修饰符来提升代码效率和封装效果。

此外，还有三个问题需要解决：
1. 模块的初始化配置依赖于`AppConfig.h`的宏，这就造成了我们不想看见的“底层需包含顶层”的现象，需要找一个办法破解这个依赖关系。
2. HAL库是C语言，`main.c`是 `.c`文件，而驱动代码是C++，App层函数不能直接用到`main.c`里。
3. 串口发送和LED也挤在App的各个函数之中，如果贸然设计成模块，其中有涉及App层的东西（模式切换）；而不分离会让代码的封装性变差。

针对这两个问题，所作的修改是：
- 每个类里额外创建一个config结构体，把初始化函数改为使用结构体配置参数的形式。App层的初始化函数先根据`AppConfig.h`初始化结构体，然后再调用各模块的初始化函数。
- 单独新建文件`WheelAppApi.c`作为API。它把C++函数套了个C语言的外壳，`main.c`只调用这里的C语言函数。
- 把串口和LED独立成APP层的模块。

# 系统结构

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

由于C不能兼容C++，`main.c` 不再持有 `App_t`。全局单例 `g_app` 位于 C++ 实现中，所有 HAL
回调和主循环均通过 `WheelApp_*` C ABI 调用。

# 模块职责

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

# 编译配置

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

# 配置与 Watch

`AppConfig.hpp` 是唯一生产参数来源，参数使用`steering::config::inline constexpr` 保存。`WheelApp` 在初始化时组装`Gm6020::Config`、`Vesc::Config`、`MasterCan::Config`、`SetpointRampConfig` 和 `WheelDebugConfig`，一次传给对应模块。

驱动不包含 `AppConfig.hpp`，也不保留产品默认 PID、斜坡或超时。配置在启动外设前校验，非法值直接进入 `Error_Handler()`。

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

#  主机测试

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

即可进行测试
