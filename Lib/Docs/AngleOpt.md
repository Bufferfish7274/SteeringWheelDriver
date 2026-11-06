# AngleOpt

## 目标

舵轮方向与底盘运动方向相差 180° 时，可以通过反转轮向 ERPM 得到完全相同的
底盘运动效果。本模块选择旋转距离更短的物理等价方案。

## 输入与输出

输入：

- 当前舵轮本地角；
- 坐标映射后的主控目标角；
- 主控 ERPM。

输出：

- 最终舵轮目标角；
- 最终 ERPM；
- 是否选择了反向方案。

## 算法

构造两个候选：

```text
A = (target_angle,          target_erpm)
B = (target_angle + 180°,  -target_erpm)
```

分别计算最短角距离：

```text
error_a = wrap180(target_a - current_angle)
error_b = wrap180(target_b - current_angle)
```

选择 `|error|` 更小的候选。`wrap180` 在计算距离时自动选择候选角的
所有 `±360°` 等价副本，因此不需要再显式添加 `target-360°` 或
`target+180°-360°`。从两个相差 180° 的候选方向中选择较短者，候选
选择时的舵向位移不会超过 90°。

## 特殊情况

- 两个候选距离相等：保留 A，避免无必要反转 ERPM；
- `target_erpm=0`：仍选择距离更近的候选；选择 B 时反向 ERPM 仍为 0；
- 输出角度始终归一化到 `[0, 360)`。

## 示例

当前 `0°`，主控目标 `100°`，ERPM 为正：

```text
A：转 100°，ERPM 为正
B：转 280°，ERPM 为负
```

选择 B，因为舵向只需移动 80°。

当前 `0°`、主控目标 `120°`、ERPM 为 0 时，同样选择 `300°`：

```text
A：转 120°，ERPM 为 0
B：转 -60°，ERPM 为 0
```

选择 B 可把舵向移动从 120° 缩短到 60°。

## 前提

- `current_angle_deg` 和 `target_angle_deg` 必须已经处于同一舵轮坐标系；
- 主控坐标到舵轮坐标的映射由应用层完成，不进入本模块；
- 本模块不处理加减速，目标角仍交给 `SetpointRamp`。

## 测试

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\AngleOpt\host\run.ps1
```
