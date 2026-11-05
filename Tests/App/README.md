# App 调试链路主机端测试

该测试使用最小 HAL/CAN 桩实例化 `App_t`，不访问真实硬件。

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\App\host\run.ps1
```

覆盖内容包括：

- 调试构建只启动 CAN1；
- 主控 CAN2 不参与调试构建；
- 模式切换停止执行器并清除使能；
- GM6020 位置、速度和直接电压目标进入统一驱动；
- VESC 目标 ERPM 限幅；
- 反馈超时清除调试使能并停止执行器。
