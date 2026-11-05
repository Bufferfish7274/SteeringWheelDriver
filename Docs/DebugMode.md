# 舵轮驱动调试模式

调试固件与正式固件使用相同的 App、GM6020、VESC 和 CAN 驱动，只替换控制
输入和安全门控。调试固件不启动 CAN2，不要求主控心跳或控制帧，但执行器
在线超时、显式使能、电压/RPM 限幅和失能零输出始终有效。

## 构建与上电

工程只有一个 Keil Target。把 `App/AppConfig.h` 中的
`WHEEL_BUILD_DEBUG` 改为 1，重新编译并烧录；改回 0 即恢复正式固件。
上电后的 500 ms 自检期间四个 LED 常亮，两个执行器保持失能。

Watch 中首先展开：

```text
g_wheel_debug
g_app
```

`g_wheel_debug.mode` 的取值：

| 值 | 模式 | 使用设备 | 目标变量 |
| --: | --- | --- | --- |
| 0 | `DISABLED` | 无 | 无 |
| 1 | `GM_POSITION_STEP` | GM6020 位置环 | `target_angle_deg` |
| 2 | `GM_POSITION_SINE` | GM6020 位置环 | `wave_amplitude_deg`、`wave_period_ms` |
| 3 | `GM_SPEED` | GM6020 速度环 | `target_speed_rpm` |
| 4 | `GM_VOLTAGE` | GM6020 直接电压 | `target_voltage_raw` |
| 5 | `VESC_SPEED` | VESC | `target_erpm` |

选择模式后，必须单独设置：

```text
g_wheel_debug.enable = 1
```

模式变化、设备反馈超时或切换回 `DISABLED` 都会把 `enable` 清零。恢复
设备后必须重新使能，不会自动继续旧目标。

## GM6020 位置阶跃

```text
g_wheel_debug.mode = 1
g_wheel_debug.target_angle_deg = 30
g_wheel_debug.direction_erpm = 100
g_wheel_debug.use_ramp = 1
g_wheel_debug.enable = 1
```

- `target_angle_deg` 是舵轮本地坐标角；
- `direction_erpm` 只参与 `AngleOpt` 的 180° 等价方向选择，不驱动 VESC；
- `direction_erpm=0` 时保持输入目标角；
- `use_ramp=1` 时目标先经过位置斜坡，0 时直接进入位置环；
- 正弦模式下 `wave_amplitude_deg` 限制到 `0..180°`，周期不小于 200 ms。

## GM6020 正弦和速度

正弦模式默认从使能时的实际位置起相：

```text
g_wheel_debug.mode = 2
g_wheel_debug.wave_amplitude_deg = 30
g_wheel_debug.wave_period_ms = 4000
g_wheel_debug.direction_erpm = 100
g_wheel_debug.use_ramp = 1
g_wheel_debug.enable = 1
```

速度环阶跃：

```text
g_wheel_debug.mode = 3
g_wheel_debug.target_speed_rpm = 50
g_wheel_debug.enable = 1
```

实际速度目标限制到 `g_app.gm6020.max_speed_rpm`，最终电压限制到
`g_app.gm6020.voltage_limit_raw`。

## GM6020 直接电压

```text
g_wheel_debug.mode = 4
g_wheel_debug.target_voltage_raw = 1000
g_wheel_debug.enable = 1
```

直发电压不经过位置环、速度环或 `voltage_sign`，适合检查功率链路和线序。
首次测试应从较小值开始，并确认机构固定、电压限幅有效。

## VESC 速度

```text
g_wheel_debug.mode = 5
g_wheel_debug.target_erpm = 100
g_wheel_debug.erpm_limit = 1000
g_wheel_debug.enable = 1
```

目标 ERPM 按 `erpm_limit` 限幅。GM6020 在该模式下保持失能零输出。

## 调参

位置和速度 PID 参数在 Watch 中直接修改：

```text
g_app.gm6020.position_pid.kp
g_app.gm6020.position_pid.ki
g_app.gm6020.position_pid.kd
g_app.gm6020.speed_pid.kp
g_app.gm6020.speed_pid.ki
g_app.gm6020.speed_pid.kd
```

运行时限幅：

```text
g_app.gm6020.max_speed_rpm
g_app.gm6020.voltage_limit_raw
g_wheel_debug.erpm_limit
```

找到合适参数后，必须停机并回填 [AppConfig.h](../App/AppConfig.h)，重新
把宏改回 0 并重新编译正式固件。Watch 修改不会写入 Flash。

## 串口

正式固件保持原有 27 列 FireWater CSV。调试固件在前 27 列之后追加：

```text
调试模式,调试使能,原始位置目标×100,方向ERPM,波形源角×100,斜坡开关,
GM目标RPM,位置P/I/D×100,速度P/I/D×100,
CAN1状态,CAN1错误码,CAN1空闲邮箱数,ERPM上限
```

角度仍按 0.01° 缩放。断开 GM6020 后应在约 20–25 ms 内看到在线位清零、
目标电压归零；恢复反馈后必须重新设置 `enable=1`。
