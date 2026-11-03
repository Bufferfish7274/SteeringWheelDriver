# 应用集成测试

本目录在主机上链接真实的 `SteeringWheelApp`、配置、GM6020、VESC、PID、
AngleOpt 和 SetpointRamp，只替换 CAN 发送与反馈注入。

分别运行正式和台架代码路径：

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\Application\host\run.ps1 -Variant production
powershell -ExecutionPolicy Bypass -File .\Tests\Application\host\run.ps1 -Variant bench
```

覆盖启动 500 ms 失能、目标一次性消费、AngleOpt 反向、首次斜坡接续、
源失效后必须重新下发目标、GM6020/VESC 在线边界、配置校验与结构性重载，
以及台架执行器最小在线集合和重新使能规则。

这些测试不验证真实 CAN 电气、NVIC、定时器抖动、机械负载或电机闭环，
相关项目仍需在台架固件上完成实机验收。
