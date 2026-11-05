# GM6020 主机端测试与调参

GM6020 驱动的通用逻辑由本目录的主机测试验证；实机调参统一使用调试固件，
步骤见 [调试模式说明](../../Docs/DebugMode.md)。

## 主机端测试

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\GM6020\host\run.ps1
```

覆盖内容包括：

- 最短角误差和单圈角度换算；
- GM6020 反馈解码、零位和方向；
- 反馈超时、PID 清零和零电压输出；
- 位置、速度和直接电压三种控制模式；
- 调参注入、运行限幅同步和抗积分饱和；
- 模式切换清使能、清目标和零输出；
- ID 1/ID 5 的 CAN 地址以及总线过滤器。

## 调试固件接口

把 `App/AppConfig.h` 的 `WHEEL_BUILD_DEBUG` 改为 1，重新编译后修改
`g_wheel_debug`：

| 模式 | 值 | 关键变量 |
| --- | --: | --- |
| GM 位置阶跃 | 1 | `target_angle_deg`、`direction_erpm`、`use_ramp` |
| GM 位置正弦 | 2 | `wave_amplitude_deg`、`wave_period_ms` |
| GM 速度 | 3 | `target_speed_rpm` |
| GM 直发电压 | 4 | `target_voltage_raw` |

选择模式后再设置 `g_wheel_debug.enable=1`。所有模式都保留反馈超时、
电压/RPM 限幅和模式切换复位。

## 首次调试建议

1. 固定舵轮机构并确认 CAN1 终端电阻和共地。
2. 使用 `DISABLED` 模式观察 `g_app.gm6020.raw_angle` 并确认零位。
3. 用小幅值确认角度方向和 `voltage_sign`。
4. 先调速度环，再调位置环。
5. 每次只修改一组参数，并记录阶跃响应、超调、稳定时间和最大电流。
6. 停机后把最终参数回填 `App/AppConfig.h`，把调试宏改回 0 并重新编译。
