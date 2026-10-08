# 跟踪云台优化阶段交接记录

记录日期：2026-10-05
项目根目录：`D:\01Project\EE\tracking_gimbal`

## 1. 本文用途

阶段 3 的基础闭环已经跑通，下一阶段进入“速度、平滑性和稳定性优化”。
本文记录优化开始前的真实状态，后续 agent 应先读本文，再读
`PROJECT_HANDOFF.md` 和 `tracking_gimbal\App\tracking.c`。

## 2. 当前结论

- K230 能识别橙色球。
- K230 通过 UART0 向 MSPM0 连续发送目标位置。
- MSPM0 能解析目标，并控制 A/B 两轴自动追踪。
- 自动追踪命令 `TRACK` 已实测成功。
- 当前方向映射正确。
- 画面上能看到跟随效果，但动作略慢，并且电机有明显的“一震一震”。
- 当前问题主要来自位置模式反复启停，不是单纯的硬件损坏。

## 3. 实机硬件结论

- K230 的串口发送脚是 `TXD(IO9)`，必须接 MSPM0 `PA31`。
  之前多次故障是因为把 IO9/IO10 接反。
- K230 与 USB 转串口不要接在同一个 USB 拓展坞。
  共用拓展坞曾造成 UART0 完全收不到数据。
- A 轴抱紧器曾经松动，导致电机转动但机械端没有完整跟随。
  修紧后 A 轴动作恢复正常。
- 当前仍然没有编码器、限位开关和自动回零。
- 上电位置就是软件零点。

## 4. 已验证方向

电机控制命令：

```text
A+ = 底座向右
A- = 底座向左
B+ = 摄像头抬头
B- = 摄像头低头
```

图像坐标映射：

```text
A+ 使目标 cy 减小
A- 使目标 cy 增大
B+ 使目标 cx 减小
B- 使目标 cx 增大
```

当前跟踪误差公式：

```c
error_a = cy - FRAME_CENTER_Y;
error_b = cx - FRAME_CENTER_X;
```

不要在没有重新做方向实验的情况下修改这些符号。

## 5. 串口与接线

| 用途 | 外设 | MSPM0 TX | MSPM0 RX | 波特率 |
|---|---|---|---:|---:|
| K230 目标数据 | UART0 | PA28 | PA31 | 115200 |
| 电脑调试信息 | UART1 | PA26 | PA25 | 115200 |

K230：

```text
K230 TXD(IO9) -> PA31
K230 GND      -> MSPM0 GND
K230 RXD(IO10) -> PA28，可选
K230 5V       -> 不接
```

电脑 USB 转串口：

```text
USB-TTL RXD -> PA26
USB-TTL TXD -> PA25
USB-TTL GND -> MSPM0 GND
```

## 6. MSPM0 当前控制方式

当前实现在 `tracking_gimbal\App\tracking.c`，使用“相对位置移动”：

```text
收到新目标
  -> 计算 cx/cy 和误差
  -> 死区判断
  -> 增益换算成角度
  -> 单次最多 2.5 度
  -> Motor_Move_Angle()
  -> 走完后停住，等待下一次控制周期
```

当前主要参数：

```text
FRAME_CENTER_X = 160
FRAME_CENTER_Y = 120
DEADZONE_A = 8 像素
DEADZONE_B = 8 像素
GAIN_FAST_A/B = 0.06
GAIN_SLOW_A/B = 0.02
MAX_ANGLE_STEP = 2.5 度
CONTROL_PERIOD_MS = 40
TRACKING_MOVE_SPEED_RPM = 35
SOFT_LIMIT_DEG = 正负 30 度
TARGET_LOST_TIMEOUT_MS = 200
```

## 7. 当前操作命令

上电后默认不自动跟踪。电脑通过 UART1 发送命令：

```text
A+      A 轴正方向移动 10 度
A-      A 轴负方向移动 10 度
B+      B 轴正方向移动 10 度
B-      B 轴负方向移动 10 度
STOP    停止自动跟踪和两轴
ZERO    把当前位置设为软件 0 度
TRACK   开启自动跟踪
RX?     查询 UART0 原始字节数和有效帧数
```

每条命令必须带换行。没有换行时命令不会执行，只会在日志中看到
没有对应的 `CMD RX` 回复。

## 8. 慢和震动的实测原因

分析 `records-2026-10-04-22-14-55.json` 后得到：

```text
目标帧间隔中位数：约 57ms
控制更新频率：约 17.5Hz
```

每次位置移动的最大角度为 2.5 度，在 35 RPM 下约 12ms 走完，
之后会停住等待下一次目标，因此形成：

```text
启动 -> 走一小段 -> 停止 -> 等待 -> 再启动
```

这会造成用户观察到的“动作略慢”和“一震一震”。当前没有加减速曲线，
开环步进本身也会有轻微振动，但主要问题在控制方式。

## 9. 推荐优化方向

推荐把跟踪执行层从“位置模式”改为“速度模式”：

```text
新目标帧
  -> 计算误差
  -> 误差乘以比例增益得到目标 RPM
  -> 限制最大 RPM
  -> 连续调用 Motor_SetAxisSpeed_RPM()
  -> 误差进入死区时停止
  -> 目标丢失或超限时立即停止
```

这样可以避免每 40ms 重复启动和刹车，预期可以同时改善：

- 跟随速度
- 运动连续性
- 抖动

速度模式必须先保留以下安全逻辑：

- A/B 当前正确方向映射
- `STOP` 和按键停止
- `TARGET_LOST_TIMEOUT_MS`
- 正负 30 度软限位
- 单轴最大 RPM 限制

## 10. 建议的后续实现顺序

1. 在 `tracking.c` 中新增速度模式，但保留位置模式作为可选回退。
2. 为 A/B 分别设置比例增益和最大 RPM。
3. 死区默认仍为 8 像素。
4. 先使用较低的最大 RPM 验证方向和无自激。
5. 再逐步增加增益和最大 RPM，记录每一次数据和现象。
6. 如果速度模式仍有明显抖动，再加入简单加减速或驱动电流调整。

## 11. 风险和禁止回退

- 不要把 K230 TXD 接回 PA28；它必须接 PA31。
- 不要把 MASTER 调试串口和 K230 接到同一根 TX/RX。
- 不要在没有新日志的情况下反转 A/B 符号。
- 不要让速度模式绕开目标丢失停止和软限位。
- Git 仓库已经存在，但只有初始提交；修改前仍建议先查看 `git diff`。

## 12. 建议阅读文件

```text
PROJECT_HANDOFF.md
tracking_gimbal\README.md
tracking_gimbal\App\tracking.c
tracking_gimbal\App\tracking.h
tracking_gimbal\Hardware\motor.c
tracking_gimbal\Hardware\control.c
```

## 13. 当前记录文件

项目内 K230 日志位于：

```text
D:\01Project\EE\tracking_gimbal\test_data
```

后续分析使用过的主要串口记录：

```text
C:\Users\25241\Downloads\records-2026-10-04-22-14-55.json
C:\Users\25241\Downloads\records-2026-10-04-22-10-59.json
```

## 14. 快速编译

```powershell
cd D:\01Project\EE\tracking_gimbal\tracking_gimbal\Debug
& 'D:\Software\ti\ccs2050\ccs\utils\bin\gmake.exe' -B -f makefile all
```

编译成功后烧录：

```text
D:\01Project\EE\tracking_gimbal\tracking_gimbal\Debug\tracking_gimbal.hex
```

## 15. 第一轮优化实现记录

记录日期：2026-10-05

本轮已经完成但尚未实机复测：

1. `tracking.c` 自动跟踪由相对位置移动改为连续速度模式。
2. 速度采用比例控制，误差乘 `0.35 RPM/像素`，最大 `30 RPM`。
3. 每 `40ms` 对 RPM 做一次斜率限制，每次最多变化 `3 RPM`。
4. `control.c` 新增统一软件 STEP 位置计数器，位置按实际发出的脉冲累计。
5. 软限位由正负 30 度调整为正负 90 度，使用实际 STEP 计数判断。
6. 新增 `POS?` 命令，可查询两轴软件位置、当前 RPM 和限位步数。
7. YbProtocol 增加功能 ID `24` 控制帧：
   `$09,24,1#` 开启跟踪，`$09,24,2#` 停止跟踪。
8. K230 `k230_tracking_ball_uart.py` 使用 `YbKey` 检测板载按键，
   短按一次切换开始和停止跟踪。

尚未完成：

- 需要烧录 MSPM0 和 K230 后实测速度、中心稳定性和软限位。
- 当前软件位置仍不是编码器位置，只能代表 MSPM0 已发出的 STEP 脉冲。
- 如果实测仍有丢步、堵转或软限位与实际机械角度偏差过大，再进入单轴编码器闭环验证。
