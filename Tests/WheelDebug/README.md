# WheelDebug 主机端测试

该测试验证 `App/WheelDebug.c` 的目标生成逻辑，不访问真实 CAN、HAL、
定时器或电机。

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\WheelDebug\host\run.ps1
```

覆盖内容包括：

- Watch 默认值和非法模式回退；
- 模式变化自动清除使能；
- 方向 ERPM 驱动的 180° 等价目标选择；
- 位置阶跃的斜坡开关；
- 正弦从当前位置起相、幅值和周期安全处理；
- 输出角度持续归一化。
