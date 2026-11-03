# 应用层与台架调参

`App/` 保存正式固件和台架固件共用的应用控制、运行配置和板级入口。

## 文件职责

| 文件 | 职责 |
|---|---|
| `steering_wheel_app.c/.h` | 输入有效性、执行器在线判断、AngleOpt、斜坡和驱动调度 |
| `app_config.c/.h` | 编译期默认值、Watch 配置、校验和应用 |
| `board_app_production.c` | MasterCan、CAN2、正式 LED/遥测 |
| `board_app_bench.c` | Watch 命令源、台架 LED/遥测 |
| `bench_input.c/.h` | 台架场景、阶跃和正弦/三角波 |
| `board_app.h` | `main.c` 使用的固定板级入口 |

正式与台架使用同一个 Keil target。默认不定义 `APP_BENCH`；在 Keil
`Options for Target -> C/C++ -> Define` 中加入 `APP_BENCH=1` 后重新编译，
即切换为台架固件。两个变体包含完全相同的应用控制核心。

## Watch 调参

修改 `g_tuning_config` 中一个或多个字段后，递增 `g_tuning_apply_seq`。
1 kHz 控制节拍会校验并应用一次请求：

- `g_tuning_apply_status=1`：应用成功；
- `g_tuning_apply_status=2`：配置非法，保持旧生效参数；
- `g_tuning_apply_status=3`：结构性配置在当前使能状态被拒绝；
- `g_tuning_apply_status=4`：CAN2 重配置失败；
- `g_tuning_applied_seq`：最近一次真正成功应用的请求序号。

PID、限幅、斜坡和超时可在线应用，默认保留 PID 积分。设备 ID、零位、
方向、传动比和主控轮号是结构性参数，必须先失能；应用后会重新初始化驱动、
重配 CAN2，并等待新的控制目标。

`g_tuning_reset_controllers` 非零时，在下一次成功应用配置后清 PID 并让斜坡
从当前 GM6020 实际角重新开始。

所有参数只保存在 RAM。掉电或复位后恢复 `AppConfig_LoadDefaults()`；
确定后的参数应写回该函数后再编译正式固件。

## 台架场景

Watch 中修改 `g_bench_input.mode`：

| 值 | 场景 |
|---:|---|
| 0 | 禁用 |
| 1 | 位置手动 |
| 2 | 位置正弦 |
| 3 | 位置三角波 |
| 4 | GM6020 速度环 |
| 5 | GM6020 直接电压 |
| 6 | VESC ERPM |

位置手动使用 `target_angle_deg` 和 `target_erpm`；自动位置使用
`wave_amplitude_deg` 和 `wave_period_ms`；GM6020 速度使用 `target_rpm`；
GM6020 电压使用 `target_voltage`；VESC 使用 `target_erpm`。

台架默认参数：

- GM6020 直接电压上限 `5000 raw`；
- VESC ERPM 限幅 `5000`；
- 自动波形幅值上限 `180°`，周期下限 `200 ms`；
- 初始模式和使能均关闭。

先设置场景和参数，再将 `g_bench_input.enable` 置 1。切换场景或执行器超时后，
必须将 `enable` 清零再置 1。GM6020 场景不要求 VESC 在线，VESC 场景不要求
GM6020 在线；位置手动模式中 ERPM 非零时才要求两个执行器在线。

`generated_angle_deg` 和 `generated_erpm` 显示当前实际送入应用核心的原始目标。

## 台架遥测

台架 CSV 依次输出：

```text
时间ms,使能,GM在线,VESC在线,需要新目标,方向反转,场景,
原始目标角×100,原始目标ERPM,映射目标角×100,最终角×100,最终ERPM,
GM实际角×100,GM实际RPM,GM目标RPM,GM目标电压,GM电流raw,
VESC实际ERPM,VESC电流mA,测试使能,配置序号,配置状态,故障标志,
斜坡输出角×100,斜坡速度
```

串口仍为非阻塞发送，上一帧未完成时跳过本次输出。
