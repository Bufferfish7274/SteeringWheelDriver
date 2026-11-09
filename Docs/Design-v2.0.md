# 舵轮分控板驱动 2.0 设计说明

## 文档信息

| 项目 | 内容 |
| --- | --- |
| 版本 | 2.0 |
| 日期 | 2026-11-06 |
| 控制器 | STM32F405RGT6 |
| 适用范围 | 单舵轮、单 GM6020、单 VESC |
| 配套文档 | [项目 README](../README.md)、[调试模式](DebugMode.md)、[模块文档](../Lib/Docs/README.md) |

本文档介绍 2.0 版本的设计目标、主要变化、软件架构、调试方式、安全模型
和版本边界。接线、调零和台架操作步骤见项目根目录 README 与调试文档。

## 1. 版本概述

2.0 在 1.0 的主控 CAN、AngleOpt、位置斜坡和 GM6020/VESC 控制基础上，
重点解决三个工程问题：

1. `main.c` 承担了过多运行逻辑，无法作为可复用的单舵轮应用库；
2. 板级测试壳和正式固件各自维护回调、遥测和电机控制流程，容易产生两套实现，而且不易进行调参；
3. 本可以简单整合测试壳和正式固件，但GM6020 调试速度环、位置环和直接电压时需要绕过正式接口直接访问 PID。

2.0 将这些职责重新分为四层：

```text
main.c
  ↓ HAL 初始化和唯一回调入口
App
  ↓ 安全门控、目标调度和执行器控制
GM6020 / VESC / MasterCan / AngleOpt / SetpointRamp
  ↓ 协议、算法和电机状态
STM32 HAL
```

版本的核心结果是：`main.c` 只负责创建 `App_t` 和调用生命周期函数，正式
控制、调试输入、LED 和串口遥测都由 App 统一调度。

## 2. 相对 1.0 的主要变化

| 项目        | 1.0                   | 2.0                       |
| --------- | --------------------- | ------------------------- |
| 顶层应用状态    | 分散在 `main.c` 的静态变量中   | 集中到 `App_t`               |
| HAL 回调    | `main.c` 直接维护控制状态     | 回调只转发给 App                |
| 正式/台架切换   | 独立板级测试壳，手工修改 main 调用  | 手动切换 `WHEEL_BUILD_DEBUG`宏 |
| GM6020 调试 | 测试代码直接操作 PID 和 CAN 发送 | 驱动提供统一三种控制模式              |
| 调试输入      | 各测试壳各自维护              | 统一使用 `g_wheel_debug`      |
| 参数位置      | 多个宏散落在 main 和库中       | 集中在 `App/AppConfig.h`     |
| 测试        | 模块主机测试和重复板级测试壳        | 模块测试、App 调试链路和调试目标生成测试    |
| 工程输出      | Keil 构建文件可能进入版本库      | `.gitignore` 排除构建产物       |


## 3. 软件结构

### 3.1 App 上下文

[App_t](../App/App.h) 是一整轮应用的持久上下文，包含：

- CAN、TIM6 和 UART HAL 句柄；
- GM6020、VESC 和位置斜坡对象；
- 运行毫秒时钟、设备在线状态和整轮使能状态；
- 最近一次控制目标、AngleOpt 结果和遥测缓冲；
- 正式版主控对象，或调试版 `WheelDebugRuntime_t`。

`App_t` 只保存数据，具体行为由以下接口完成：

| 接口 | 职责 |
| --- | --- |
| `App_Init()` | 初始化驱动、参数、输入层和 UART 中断 |
| `App_Start()` | 启动 CAN1、正式版 CAN2 和 TIM6 |
| `App_Loop()` | 调度 LED 和 50 Hz 串口遥测 |
| `App_OnCanRx()` | 排空 CAN RX FIFO 并分发反馈/主控帧 |
| `App_OnTimerElapsed()` | 执行 1 kHz 实时控制 |
| `App_OnUartTxComplete()` | 释放下一次遥测发送许可 |

### 3.2 main.c

[main.c](../Core/Src/main.c) 负责：

1. HAL、系统时钟和 CubeMX 外设初始化；
2. 创建 `static App_t g_app`；
3. 调用 `App_Init()`、`App_Start()` 和主循环 `App_Loop()`；
4. 提供唯一的 CAN、TIM 和 UART HAL 回调，并转发给 App。

电机使能、PID、遥测格式和调试模式判断均不再写在 `main.c` 中。

## 4. 正式与调试输入

工程只保留一个 Keil Target：

```text
SteeringWheelDriver
```

在 [AppConfig.h](../App/AppConfig.h) 中修改：

```c
#define WHEEL_BUILD_DEBUG 0
```

- `0`：正式固件，启动 CAN1 和 CAN2，接收主控命令；
- `1`：调试固件，只启动 CAN1，通过 Watch 修改 `g_wheel_debug`。

切换后重新编译并烧录同一个 Target。单 Target 中同时包含 `MasterCan.c` 和
`WheelDebug.c`，但 `App.c` 会通过编译宏关闭另一条运行路径。调试版不启动
CAN2，也不要求主控心跳或控制帧。

## 5. 正式控制链路

```text
CAN2 主控控制帧
  → MasterCan 解码
  → 主控角度 sign/offset 映射
  → AngleOpt 选择更近的物理等价舵向
  → SetpointRamp 平滑位置目标
  → GM6020 位置-速度串级 PID
  → CAN1 电压控制帧

主控 ERPM
  → AngleOpt 原样使用或取反
  → VESC SET_RPM
```

主控目标只在收到新控制帧时重新计算 AngleOpt。后续周期持续使用该目标，
避免当前舵向变化导致 1 kHz 循环内方向反复跳变。

正式模式采用整轮安全条件：主控链路有效、已收到控制命令、GM6020 在线、
VESC 在线。任一条件失效时两个执行器立即失能并持续发送零输出。

## 6. GM6020 控制接口

2.0 在 [GM6020.h](../Lib/GM6020.h) 中提供统一控制模式：

| 模式 | 控制路径 | 用途 |
| --- | --- | --- |
| `GM6020_CONTROL_POSITION` | 位置 PID → 目标 RPM → 速度 PID | 正式舵向控制 |
| `GM6020_CONTROL_SPEED` | 目标 RPM → 速度 PID | 速度环调参 |
| `GM6020_CONTROL_VOLTAGE` | 目标 raw 电压 → 限幅 | 功率链路线序测试 |

目标通过 `GM6020_SetTargetAngle()`、`GM6020_SetTargetSpeed()` 和
`GM6020_SetTargetVoltage()` 写入，由 `GM6020_ControlStep()` 统一分派。
调试代码不再直接调用 PID 或手工发送 GM6020 CAN 帧。

模式和反馈超时按固定优先级处理：

1. 反馈超时：清使能、清目标、复位 PID、发送零电压；
2. 未使能或离线：持续发送零电压；
3. 在线使能：按位置、速度或直接电压模式计算输出。

直接电压模式故意不应用 `voltage_sign`，发送的是经过运行限幅后的 raw 值。

## 7. 调试模式

调试输入集中在 [WheelDebug.h](../App/WheelDebug.h) 的 `g_wheel_debug`：

| 值 | 模式 | 使用设备 |
| --: | --- | --- |
| 0 | `DISABLED` | 无，两个执行器零输出 |
| 1 | `GM_POSITION_STEP` | GM6020 位置环 |
| 2 | `GM_POSITION_SINE` | GM6020 位置环 |
| 3 | `GM_SPEED` | GM6020 速度环 |
| 4 | `GM_VOLTAGE` | GM6020 直接电压 |
| 5 | `VESC_SPEED` | VESC ERPM |

位置阶跃和正弦模式始终经过 AngleOpt，并通过 `direction_erpm` 决定是否
允许选择 180° 等价方向。`use_ramp` 控制是否经过 SetpointRamp。

安全行为：

- 默认模式为 `DISABLED`，默认使能为 0；
- 模式变化会自动清除使能并停止执行器；
- 当前模式只需要目标设备在线，不要求另一台设备同时连接；
- 设备掉线或反馈超时时自动清除调试使能；
- 恢复设备后必须重新设置 `enable=1`；
- 正弦幅值限制为 `0..180°`，周期不小于 `200 ms`。

完整 Watch 操作和调参流程见 [调试模式](DebugMode.md)。

## 8. 实时运行与遥测

| 事件 | 周期/触发 | 执行内容 |
| --- | --- | --- |
| CAN RX 中断 | 反馈或控制帧到达 | 解码反馈，登记新主控命令 |
| TIM6 | 1 kHz | 在线判断、安全处理、目标更新、PID 和 CAN 发送 |
| 主循环 LED | 10 ms | 刷新在线和使能状态 |
| UART 遥测 | 20 ms | 50 Hz 非阻塞发送 |

正式固件保持 27 列 FireWater CSV。调试固件保持前 27 列不变，并在末尾
追加调试模式、目标、方向 ERPM、波形源角、GM 目标 RPM、两组 PID 的
P/I/D 中间项、CAN1 状态和 ERPM 限幅。

UART 忙时丢弃本次遥测，不阻塞 1 kHz 控制中断。

## 9. 配置与调参

应用配置集中在 [AppConfig.h](../App/AppConfig.h)，包含：

- 主控轮号、GM6020 ID 和 VESC ID；
- GM6020 零位、角度方向、电压方向和传动比；
- 位置/速度 PID 参数和积分策略；
- 最大 RPM、电压 raw 限幅和反馈超时；
- 位置斜坡速度和加速度上限；
- 调试模式默认目标和安全限幅；
- LED、自检和遥测周期。

`AppConfig.h` 是生产参数的唯一来源。`App.c` 在启动时把宏组装成
`GM6020_Config_t`、`SetpointRamp_Config_t`、`VESC_Config_t` 等类型化
对象并单次注入对应模块。驱动库不包含 APP 头文件，也不保留产品默认 PID、
默认斜坡或默认超时；非法配置在启动阶段直接进入 `Error_Handler()`。

正式路径和调试路径共用同一个控制周期与位置斜坡对象。Watch 修改的是
运行时对象，不会形成第二套配置来源。

调参流程：

1. 将 `WHEEL_BUILD_DEBUG` 设为 1 并重新编译；
2. 在 Watch 中调整目标、PID 和限幅；
3. 通过串口记录阶跃或正弦响应；
4. 停机并把最终参数回填 `AppConfig.h`；
5. 将 `WHEEL_BUILD_DEBUG` 改回 0；
6. 重新编译正式固件并完成主控联调。

Watch 参数只存在于 RAM，不会掉电保存。

## 10. 测试与验证

2.0 保留并更新模块主机测试，同时新增：

| 测试 | 覆盖内容 |
| --- | --- |
| `Tests/App` | 调试 App 的 CAN1 启动、模式切换、目标下发和断线停机 |
| `Tests/WheelDebug` | 模式检测、180° 方向选择、正弦起相和斜坡开关 |
| `Tests/GM6020` | 位置/速度/电压模式、配置注入和超时零输出 |
| `Tests/MasterCan` | 控制帧、心跳、持久目标和超时恢复 |
| `Tests/AngleOpt` | 最近物理等价舵向选择 |
| `Tests/SetpointRamp` | 位置斜坡速度和加速度限制 |
| `Tests/VESC` | SET_RPM、STATUS 解码和超时零输出 |

主机测试覆盖纯逻辑和 App 调试链路。真实 CAN、电机、功率桥、LED 和机械
动态仍需按调试文档完成台架验收。

## 11. 兼容性与迁移

- 主控 CAN2 控制/心跳协议保持不变；
- GM6020 和 VESC 的 CAN ID、DLC 与数据布局保持不变；
- 正式版前 27 列串口遥测保持不变；
- 硬件接线和 CAN 波特率保持不变；
- 配置从 `main.c` 宏迁移到 `App/AppConfig.h`；
- 旧板级测试壳已退役，实机调参统一使用调试固件；
- 旧版代码若直接访问 GM6020 PID 或手工发送控制帧，需要改用 2.0 公共接口。

## 12. 当前边界

- 只支持一个舵轮、一个 GM6020 和一个 VESC；
- GM6020 只支持单圈绝对角；
- 当前机械方案为直驱，传动比暂按 1.0 使用；
- 参数是编译期配置，不支持串口在线写 Flash；
- 不向主控回传舵轮状态；
- 调试固件面向受控台架，不能替代正式安全联调；
- VESC 内部完成轮向速度闭环，分控板只负责 CAN 目标下发和状态观察；
- 调试使能、反馈超时和限幅不会因为绕过主控而被取消。

## 13. 后续方向

- 增加配置版本号和参数校验；
- 增加 GM6020 温度、电流和长时间堵转保护；
- 增加 VESC 完整故障码和 CAN Bus-Off 诊断；
- 增加主控状态回传；
- 为减速传动增加多圈角度处理；
- 将更多控制逻辑拆分为可独立主机测试的模块。
- 优化AngleOpt算法
- 改为C++以提升封装性
