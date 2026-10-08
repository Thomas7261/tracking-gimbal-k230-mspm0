# 跟踪云台初步结项交接记录

记录日期：2026-10-06
项目根目录：`D:\01Project\EE\tracking_gimbal`

## 1. 本文用途

本文用于“教学小 Demo 初步结项”。

它记录当前已经完成的功能、实机验证结果、尚未验证的改动、已知限制、
接线方式、关键参数和后续恢复入口。后续继续开发时，应先阅读本文，
再阅读 `PROJECT_HANDOFF.md` 和 `OPTIMIZATION_HANDOFF.md`。

## 2. 当前结论

截至 2026-10-06，项目已经达到教学 Demo 的基础目标：

- K230 能识别橙色球并输出目标位置。
- K230 与 MSPM0 的 UART0 通信稳定。
- MSPM0 能控制 A/B 两轴持续跟踪。
- 方向映射已经实机确认。
- 速度模式比原来位置启停模式明显更连续、更丝滑。
- 中心死区计算正常。
- K230 板载按键可以切换跟踪开始和停止。
- 目标丢失后 MSPM0 会按超时保护停止。
- 系统可以脱离电脑串口独立运行。

目前还保留一个算法调优项：

- 小球快速移动后急停时，仍可能出现一定过度调位。
- 已经加入软件制动和误差变化率前瞻，但结果尚未完成实机复测。

项目没有编码器、限位开关和自动回零，因此定位仍属于开环估计。

## 3. 实机经验结论

### 3.1 USB 转串口干扰

调试期间出现过“接上 USB 转串口后行为异常”的情况。最终确认是串口
模块本身带来的干扰，拔掉串口模块后系统可以独立正常运行。

后续排查类似问题时，应优先检查：

- USB 转串口模块是否只是用于观察，不应改变系统动作。
- MSPM0、K230、D36A 和电机电源是否保持良好的公共地。
- 调试串口模块是否引入了额外供电、地环路或复位干扰。

### 3.2 当前运行方式

正常演示时不依赖电脑：

```text
K230 上电并自动运行 main.py
MSPM0 上电并保持停止状态
短按 K230 板载按键
MSPM0 收到控制帧后进入 TRACK
移动橙色球进行跟踪
再次短按按键停止
```

## 4. 硬件组成

| 模块 | 型号或说明 |
|---|---|
| 视觉模块 | 亚博智能 K230 |
| 主控 | TI MSPM0G3507 |
| 步进驱动 | WHEELTEC D36A 双路驱动器 |
| 云台 | WHEELTEC 二维步进云台 |
| 目标 | 橙色球 |
| 调试 | USB 转 TTL 串口模块，仅调试时需要 |

当前没有使用：

- 编码器。
- 限位开关。
- 自动回零。
- IMU 或外部位置传感器。

## 5. 接线摘要

### 5.1 K230 到 MSPM0

| K230 | MSPM0G3507 | 说明 |
|---|---|---|
| TXD(IO9) | PA31 | UART0 RX |
| RXD(IO10) | PA28 | UART0 TX，可选 |
| GND | GND | 必须共地 |
| 5V | 不接 | 不要向控制板倒灌 |

### 5.2 电脑调试串口

| USB 转串口 | MSPM0G3507 |
|---|---|
| RXD | PA26，UART1 TX |
| TXD | PA25，UART1 RX |
| GND | GND |

调试串口参数：

```text
115200
8 数据位
1 停止位
无校验
```

### 5.3 D36A 到 MSPM0

| D36A | MSPM0G3507 | 功能 |
|---|---|---|
| ST1 | PA24 | A 轴 STEP |
| DIR1 | PA13 | A 轴方向 |
| EN1 | PA16 | A 轴使能 |
| ST2 | PA22 | B 轴 STEP |
| DIR2 | PA14 | B 轴方向 |
| EN2 | PA17 | B 轴使能 |
| GND | GND | 必须共地 |

## 6. 当前软件结构

| 文件 | 职责 |
|---|---|
| `k230_tracking_ball_uart.py` | 识别橙色球、显示预览、发送目标帧和按键控制帧 |
| `tracking_gimbal/empty.c` | 系统初始化和主循环调度 |
| `App/usart.c` | UART0 接收中断、UART1 调试收发 |
| `App/yb_protocol.c` | 解析目标帧和控制帧 |
| `App/tracking.c` | 视觉误差、比例速度、制动、软限位和安全停止 |
| `Hardware/control.c` | 位置模式、软件位置和 STEP 计数 |
| `Hardware/motor.c` | PWM、DIR、EN 和 STEP 脉冲输出 |

## 7. K230 目标帧和控制帧

目标位置帧：

```text
$23,01,020,030,040,050#
```

含义：

```text
x=20 y=30 w=40 h=50
```

K230 按键控制帧：

```text
$09,24,1#
$09,24,2#
```

含义：

```text
1 = 开始跟踪
2 = 停止跟踪
```

## 8. MSPM0 调试命令

每条命令都需要换行：

```text
A+       A 轴正向移动 10 度
A-       A 轴反向移动 10 度
B+       B 轴正向移动 10 度
B-       B 轴反向移动 10 度
STOP     停止自动跟踪和两轴
TRACK    开启自动跟踪
ZERO     把当前位置设为软件 0 度
RX?      查看 UART0 字节数和有效帧数
POS?     查看软件位置、RPM、误差变化率和限位步数
```

## 9. 当前主要参数

### 9.1 K230

```text
DETECT_WIDTH = 320
DETECT_HEIGHT = 240
SENSOR_WIDTH = 1280
SENSOR_HEIGHT = 960
UART_SEND_INTERVAL_MS = 40
KEY_DEBOUNCE_MS = 250
LOST_FRAME_LIMIT = 5
MIN_CONTOUR_AREA = 120
MIN_CIRCULARITY = 0.45
```

### 9.2 MSPM0

```text
FRAME_CENTER_X = 160
FRAME_CENTER_Y = 120
DEADZONE_A/B = 8 像素
SPEED_GAIN_A/B = 0.35 RPM/像素
MAX_TRACKING_RPM_A/B = 30 RPM
MIN_TRACKING_RPM = 3 RPM
CONTROL_PERIOD_MS = 40
TARGET_LOST_TIMEOUT_MS = 200
SOFT_LIMIT_DEG = 正负 90 度
SPEED_ACCEL_RPM_PER_CYCLE = 3 RPM
SPEED_BRAKE_RPM_PER_CYCLE = 8 RPM
ERROR_RATE_LOOKAHEAD_S = 0.08 秒
ERROR_RATE_FILTER_ALPHA = 0.35
ERROR_RATE_LIMIT_PX_PER_S = 600
```

## 10. 过冲抑制说明

急停过冲主要来自：

- K230 图像处理和 UART 传输存在延迟。
- 云台机械结构有惯性。
- 原始比例控制在误差刚开始变小时仍保持较高速度。

当前软件增加了两个处理：

1. 统计误差变化率。
2. 用约 80ms 的前瞻提前降低目标速度。

同时把减速斜率设置为 `8 RPM/40ms`，比加速的 `3 RPM/40ms` 更快。

该算法属于“视觉闭环中的 P 控制加 D 类制动”，不需要编码器。
它可以改善过冲，但不能完全消除机械惯性和视觉延迟。

## 11. K230 预览画面开关

文件：

```text
D:\01Project\EE\tracking_gimbal\k230_tracking_ball_uart.py
```

开关：

```python
PREVIEW_SCALE_ENABLE = True
PREVIEW_INTERPOLATION = "linear"
```

可选值：

```text
PREVIEW_SCALE_ENABLE:
    True  = 放大到 640x480
    False = 保持 320x240

PREVIEW_INTERPOLATION:
    "linear"  = 画面更平滑，计算量稍大
    "nearest" = 运算更快，边缘更像素化
```

预览放大只影响显示，不影响目标坐标、串口数据和控制参数。

## 12. 编译和运行

### 12.1 MSPM0

```powershell
cd D:\01Project\EE\tracking_gimbal\tracking_gimbal\Debug
& 'D:\Software\ti\ccs2050\ccs\utils\bin\gmake.exe' -B -f makefile all
```

输出：

```text
D:\01Project\EE\tracking_gimbal\tracking_gimbal\Debug\tracking_gimbal.hex
```

### 12.2 K230

1. 用 CanMV IDE 打开 `k230_tracking_ball_uart.py`。
2. 保存到 K230 的 `main.py`。
3. 断开 IDE 数据线并重新上电。
4. 检查 LCD 是否显示 `TRACK OFF`。
5. 短按板载按键，确认显示变为 `TRACK ON`。

## 13. Git 状态

已存在的提交：

```text
730af64  feat: smooth visual tracking and add k230 key control
966e34b  chore: mark pre-optimization baseline
d4a612d  初始化项目：简单跟踪云台基线
```

当前工作区还可能包含尚未提交的改动：

- 过冲抑制算法。
- K230 预览放大开关。
- 核心文件学习注释。

结项前应先执行：

```powershell
git status
git diff
```

确认测试结果后，再决定提交或回退。

## 14. 已知限制

- 没有编码器，`POS?` 显示的是已发 STEP 估计值，不是机械真实角度。
- 没有限位开关和自动回零，上电位置就是软件零点。
- 软限位不能识别掉电后的人工移动或严重丢步。
- 颜色阈值依赖灯光、背景和曝光。
- 快速目标急停仍可能需要继续调参。
- 预览放大使用软件缩放，可能轻微降低 K230 帧率。
- D36A 细分和代码必须保持一致。

## 15. 结项前建议完成的最小验证

- [ ] 烧录当前 MSPM0 代码并编译通过。
- [ ] K230 脚本保存为 `main.py`，离线自动运行。
- [ ] 短按 K230 按键能够稳定开始和停止。
- [ ] 小球慢速移动时动作平滑。
- [ ] 小球快速移动后急停，记录是否仍然过冲。
- [ ] 连续运行 5 分钟，观察串口是否有错误帧。
- [ ] 发送 `POS?`，确认限位步数为 800。
- [ ] 确认拔掉电脑调试模块后系统仍能独立运行。

## 16. 后续继续开发的入口

如果后续继续优化，建议只选择一个方向：

1. 优先调软件过冲参数，不增加硬件。
2. 如果确实存在明显丢步，再给单轴添加 MS42CG 编码器验证。
3. 编码器验证成功后再考虑第二轴，不建议直接一步做双编码器。

## 17. 关键文件

```text
k230_tracking_ball_uart.py
PROJECT_HANDOFF.md
OPTIMIZATION_HANDOFF.md
tracking_gimbal\App\tracking.c
tracking_gimbal\App\tracking.h
tracking_gimbal\App\yb_protocol.c
tracking_gimbal\App\yb_protocol.h
tracking_gimbal\Hardware\control.c
tracking_gimbal\Hardware\control.h
tracking_gimbal\Hardware\motor.c
tracking_gimbal\Hardware\motor.h
```
