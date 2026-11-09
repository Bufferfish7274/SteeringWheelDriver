# App 集成主机端测试

该测试使用最小 HAL/CAN 桩共享全局 `g_app`，不访问真实硬件。

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\App\host\run.ps1
```

覆盖内容包括：

- 调试构建只启动 CAN1，正式构建启动 CAN1 和 CAN2；
- 配置从 `AppConfig.hpp` 单次注入驱动和位置斜坡；
- 模式切换停止执行器并清除使能；
- GM6020 位置、速度和直接电压目标进入统一驱动；
- VESC 目标 ERPM 限幅；
- 正式控制在新命令到达后同时使能整轮；
- 反馈超时清除调试/正式使能并停止执行器；
- 正式恢复后要求新的主控命令。
