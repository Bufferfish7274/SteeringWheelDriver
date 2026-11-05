# 舵轮分控板驱动

基于 STM32F405RGT6 的单舵轮控制固件。分控板通过 CAN2 接收主控目标，
通过 CAN1 控制 GM6020 舵向电机和 VESC 轮向驱动，并输出 UART 调试遥测。

1.1 设计思路、功能范围和版本边界见
[Docs/Design-v1.0.md](Docs/Design-v1.0.md)。

## 系统结构

```text
主控板
  │ CAN2：0x101 控制 / 0x111 心跳
  ▼
舵轮分控板
  ├─ App：组合对象、安全状态机、构建模式调度
  ├─ MasterCan：协议解码、控制可用、链路超时
  ├─ SteeringWheelApp：安全门控、AngleOpt、斜坡和驱动调度
  ├─ AngleOpt：选择最短物理等效舵向，并决定 ERPM 符号
  ├─ SetpointRamp：限制 GM6020 目标角速度和角加速度
  ├─ GM6020：舵向位置环
  └─ VESC：轮向速度环
        │ CAN1：本地电机总线
        ├─ GM6020 反馈/电压指令
        └─ VESC 状态/ERPM 指令
```

## 硬件连接

| 接口 | 引脚 | 用途 |
|---|---|---|
| CAN1 | PA11/PA12 | GM6020 与 VESC，本地电机总线 |
| CAN2 | PB12/PB13 | 主控总线 |
| USART1 | PA9/PA10 | 115200、8N1 调试遥测 |
| LED | PB4～PB7 | 低电平点亮 |

两路 CAN 均为经典 CAN，波特率 `1 Mbps`。总线两端应各有 `120 Ω`
终端电阻，所有节点必须共地，且 CAN_H/CAN_L 不得反接。

## 运行配置

| 项目 | 当前默认值 |
|---|---|
| 主控轮号 | 1 |
| GM6020 ID | 1 |
| GM6020 机械零位原始角 | 1386 |
| GM6020 角度/电压方向 | `+1` / `+1` |
| GM6020 传动比 | 1.0 |
| VESC 控制器 ID | 67 |
| 主控角度映射 | `sign=+1`、`offset=0°` |
| 位置 PID | `20 / 2 / 0.1` |
| 速度 PID | `70 / 50 / 0` |
| 最大角速度/角加速度 | `790 °/s` / `4000 °/s²` |

配置集中在 [AppConfig.h](App/AppConfig.h)。修改机械零位、设备 ID、
PID、斜坡或安全限幅时，应优先修改这些宏，不要在业务函数中写死。

`g_tuning_apply_status` 和 `g_tuning_applied_seq` 分别表示最近一次应用结果
和实际生效的请求序号。参数只保存在 RAM，掉电后恢复编译期默认值；接受一组
参数后应将其写回 `AppConfig_LoadDefaults()`。

PID、限幅、斜坡和超时可在运行中应用，默认保留 PID 积分状态。设备 ID、
零位、方向和传动比属于结构性参数，只能在整轮失能时应用；此时会重新初始化
驱动并重配 CAN2 过滤，随后必须收到新目标。

## 正式与台架构建

工程只有一个 Keil target，默认不定义 `APP_BENCH`，编译正式固件。需要在
Keil 的 C/C++ 预处理器定义中加入 `APP_BENCH=1` 后重新编译，即生成台架固件。

两种固件共用 `SteeringWheelApp`、AngleOpt、斜坡和 GM6020/VESC 驱动：

- 正式入口从 CAN2 的 MasterCan 命令产生目标；
- 台架入口从 Keil Watch 和波形产生目标，不启动 CAN2。

台架入口提供 `DISABLED`、位置手动、位置正弦、位置三角波、GM6020 速度、
GM6020 直接电压和 VESC ERPM。切换模式或发生执行器故障后必须先将
`g_bench_input.enable` 清零再置 1，才允许重新使能。

## 模块文档

各生产模块的接口、算法和约束见 [Lib/Docs/README.md](Lib/Docs/README.md)。
应用编排、构建切换和 Watch 调参说明见 [App/README.md](App/README.md)。

## 启动顺序

`main.c` 只保留外设初始化和 App 调用。启动顺序为：

1. 初始化 GPIO、CAN1、CAN2、TIM6 和 USART1；
2. `App_Init()` 初始化 GM6020、VESC、位置斜坡和构建对应的输入层；
3. `App_Start()` 启动 CAN1，正式版继续启动 CAN2 并只接收 `0x101`/`0x111`；
4. 最后启动 TIM6 1 kHz 中断；
5. 主循环只调用 `App_Loop()`，由 App 调度 LED 和串口遥测。

上电后前 500 ms 四个 LED 常亮，同时禁止两个执行器使能。

## 运行时行为

安全逻辑不使用独立状态机框架，而是由启动时间、输入有效性和执行器在线条件
直接决定使能：

- 启动后前 500 ms 无论是否收到目标，GM6020 和 VESC 都保持零输出；
- 正式模式要求主控链路有效、已收到控制目标且两个执行器在线；
- 条件失效时两个执行器立即失能并持续发送零输出；
- 故障期间收到的目标会被丢弃，恢复后必须重新收到新目标。

控制命令的数据流：

```text
主控目标角
  → 坐标映射
  → AngleOpt 选择最终舵向与 ERPM 符号
  → GM6020 位置斜坡 + 位置环
  → VESC 内置速度环
```

角度候选只在收到新控制帧时计算一次，避免当前角变化导致候选方向在
1 kHz 循环中反复切换。

## 主控 CAN 协议

### 控制帧

- 标准数据帧；
- ID：`0x100 + wheel_id`，轮号 1 为 `0x101`；
- DLC：8；
- 主控只在 ERPM 或角度目标变化时发送。

| 字节 | 内容 |
|---|---|
| `Data[0..3]` | 大端 `int32 ERPM` |
| `Data[4..7]` | 大端角度码，范围 `0..16383` |

### 心跳帧

- 标准数据帧；
- ID：`0x110 + wheel_id`，轮号 1 为 `0x111`；
- DLC：8，负载全 0；
- 建议主控每 80 ms 发送一次。

控制目标在链路正常期间持续有效。链路超过 500 ms 无控制帧或心跳后，
分控板清除 `control_seen` 并进入整轮故障状态。

## 本地 CAN 设备

### GM6020

- 当前使用旧版固件默认电压控制模式；
- ID 1 的控制帧为 `0x1FF`；
- 反馈帧为 `0x204 + ID`，ID 1 对应 `0x205`；
- 反馈为 1 kHz；
- 机械角范围 `0..8191`；
- 软件零位为 `1386`，在该位置换算后的舵轮角约为 `0°`。

### VESC

- 扩展帧；
- `SET_RPM`：`(3 << 8) | 67 = 0x343`；
- `STATUS`：`(9 << 8) | 67 = 0x943`；
- VESC 状态建议 50 Hz；
- 驱动超时为 100 ms；
- 分控板每 1 ms 更新一次 ERPM 指令。

## 角度与斜坡

`AngleOpt` 在两个物理等价方向中选择距离当前舵向更近的方案：

```text
A: (target_angle,  target_erpm)
B: (target_angle + 180°, -target_erpm)
```

例如当前 `0°`、目标 `100°`、ERPM 为正时，方案 B 只需转到 `280°`
并反转 ERPM，因此会选择 B。

GM6020 目标角经过位置斜坡：

- 最大角速度：`790 °/s`；
- 最大角加速度：`4000 °/s²`。

斜坡只作用于 GM6020 目标角。VESC ERPM 按主控命令直接输出，不在本模块
再次限幅。

## 机械零位校准

GM6020 原始角范围为 `0..8191`。修改
`g_tuning_config.gm6020_zero_raw` 或编译期默认零位后，反馈角和目标角会
同时以新位置为 `0°`。

推荐流程：

1. 给分控板供电，但不连接主控，确保电机失能；
2. 将舵轮转到机械零位；
3. 在 Keil Watch 中读取 `g_app.gm6020.raw_angle`；
4. 将 [AppConfig.h](App/AppConfig.h) 中 `APP_GM6020_ZERO_RAW` 改为该值；
5. 重新编译、烧录；
6. 保持主控断开，确认 GM6020 实际角显示接近 `0°`；
7. 正反旋转舵轮，确认角度变化方向正确。

如果串口只能显示 GM6020 实际角×100，而当前零位为 `old_zero`：

```text
delta_raw = round(displayed_angle_deg × 8192 / 360)
new_zero  = (old_zero + delta_raw) mod 8192
```

## 安全策略

正式模式下，以下任一条件不满足时，GM6020 和 VESC 都失能并持续发送零输出：

- 主控 `link_valid=1`；
- 主控 `control_seen=1`；
- GM6020 在 20 ms 超时窗口内在线；
- VESC 在 100 ms 超时窗口内在线。

故障后的恢复要求：

1. 主控链路和执行器全部恢复在线；
2. 收到一帧新的有效控制命令；
3. 角度候选重新选择后，两个执行器再次同时使能。

台架模式不要求主控在线，只检查当前场景涉及的执行器。GM6020 位置/速度/
电压场景只需要 GM6020 在线，VESC ERPM 场景只需要 VESC 在线。模式切换或
执行器掉线后，必须把 `g_bench_input.enable` 清零再置 1。

## LED

所有 LED 均为低电平点亮。

| LED | 含义        |
| --- | --------- |
| PB4 | 主控链路有效    |
| PB5 | VESC 在线   |
| PB6 | 整轮已使能     |
| PB7 | GM6020 在线 |

任一执行器或主控失效时，对应在线灯熄灭，同时 PB6 熄灭。

## UART 遥测

USART1 使用 `115200、8N1`，每 20 ms 尝试发送一次，即 50 Hz。
串口为非阻塞发送；若上一帧尚未完成，则跳过本次遥测。

输出为 FireWater 兼容的 27 列 CSV：

|  通道 | 内容                 |
| --: | ------------------ |
|   1 | 时间，ms              |
|   2 | 主控 `link_valid`    |
|   3 | 主控 `control_seen`  |
|   4 | GM6020 在线          |
|   5 | VESC 在线            |
|   6 | 整轮活动               |
|   7 | 方向反转标志             |
|   8 | 需要新控制帧             |
|   9 | 主控 ERPM            |
|  10 | 主控角度码              |
|  11 | 主控目标角×100          |
|  12 | 坐标映射后目标角×100       |
|  13 | AngleOpt 最终舵向角×100 |
|  14 | AngleOpt 最终 ERPM   |
|  15 | GM6020 实际角×100     |
|  16 | GM6020 转速，RPM      |
|  17 | GM6020 转矩电流 raw    |
|  18 | GM6020 温度          |
|  19 | GM6020 目标电压 raw    |
|  20 | VESC 实际 ERPM       |
|  21 | VESC 实际电流，mA       |
|  22 | VESC 占空比×1000      |
|  23 | 位置斜坡输出角×100        |
|  24 | 位置斜坡速度，°/s         |
|  25 | 距上一控制帧的时间，ms       |
|  26 | 距上一心跳帧的时间，ms       |
|  27 | 故障标志               |
|     |                    |

调试固件保持以上 27 列不变，并在末尾追加调试模式、使能、位置源角、
方向 ERPM、GM 目标 RPM、PID 中间项、CAN1 状态和 ERPM 上限。字段定义见
[调试模式说明](Docs/DebugMode.md)。

### 故障标志

| 位 | 值 | 含义 |
|---:|---:|---|
| bit0 | 1 | 主控链路无效 |
| bit1 | 2 | 尚未收到控制目标 |
| bit2 | 4 | GM6020 离线 |
| bit3 | 8 | VESC 离线 |
| bit4 | 16 | 需要新控制帧 |
| bit5 | 32 | GM6020 CAN 发送状态异常 |
| bit6 | 64 | VESC CAN 发送状态异常 |
| bit7 | 128 | 运行配置应用失败 |

台架固件使用独立 CSV 通道顺序，至少包含时间、使能、执行器在线、场景、
原始目标、AngleOpt 结果、GM6020/VESC 反馈、配置应用序号、配置应用状态和
故障标志；生成台架固件后以当前 `board_app_bench.c` 的格式为准。

## 编译与烧录

Keil 工程：

```text
MDK-ARM/SteeringWheelDriver.uvprojx
```

工程只有一个构建目标 `SteeringWheelDriver`。正式和调试固件通过
[AppConfig.h](App/AppConfig.h) 顶部的 `WHEEL_BUILD_DEBUG` 切换：

- `WHEEL_BUILD_DEBUG=0`：正式主控链路；
- `WHEEL_BUILD_DEBUG=1`：只启动 CAN1，通过 Watch 中的
  `g_wheel_debug` 选择调试模式。

修改宏后重新编译并烧录同一个目标。不要在 Keil 的 Preprocessor Symbols
中重复定义该宏。

测试和烧录步骤见 [调试模式说明](Docs/DebugMode.md)。板级测试不再使用
独立测试壳；CAN、TIM6、USART1 回调统一由 `main.c` 转发给 App。

## 主机端测试

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\App\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\MasterCan\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\AngleOpt\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\SetpointRamp\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\GM6020\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\VESC\host\run.ps1
powershell -ExecutionPolicy Bypass -File .\Tests\WheelDebug\host\run.ps1
```

各模块的协议和主机测试见对应目录下的 README；实机调参统一按
[调试模式说明](Docs/DebugMode.md) 执行。

## 常见故障

- 主控链路灯不亮：检查 CAN2 接线、1 Mbps、120 Ω 和主控心跳。
- GM6020 灯不亮：检查 CAN1、GM6020 ID、供电和反馈帧。
- VESC 灯不亮：检查 VESC ID、CAN1、状态帧频率和 VESC 通讯超时设置。
- 整轮不使能：需要同时满足主控控制可用、GM6020 在线和 VESC 在线。
- 角度方向错误：先检查 GM6020 `angle_sign`，不要通过修改机械零位掩盖方向问题。
- 主控断联恢复后不动：这是安全行为，必须重新下发控制帧。
