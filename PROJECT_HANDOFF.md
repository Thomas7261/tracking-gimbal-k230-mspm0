# 跟踪云台项目交接记录

记录日期：2026-10-05
项目根目录：`D:\01Project\EE\tracking_gimbal`

## 1. 项目目标

使用 K230 识别橙色球，通过串口把目标位置发送给
MSPM0G3507，再由 MSPM0 控制二维步进云台进行跟踪。

当前系统链路：

```text
K230 摄像头
  -> 颜色分割与目标定位
  -> YbProtocol 串口帧
  -> MSPM0G3507 UART0
  -> 解析目标位置
  -> 后续由 tracking.c 控制云台
```

当前项目用于学习和验证视觉与云台控制联动，不追求最终比赛版本的
高级功能。

教学 Demo 的初步结项记录见：

```text
D:\01Project\EE\tracking_gimbal\PROJECT_CLOSEOUT_HANDOFF.md
```

## 2. 当前阶段状态

| 阶段 | 状态 | 说明 |
|---|---|---|
| 阶段 0：硬件与电机底座 | 已完成 | 02_Gimbal_PosiCtrl 已实测可运行 |
| 阶段 1：MSPM0 UART 接收 | 已完成 | 能解析 `$23,01,020,030,040,050#` |
| 阶段 2：K230 识别橙色球 | 已完成 | 免校准固定阈值运行稳定 |
| 阶段 3：K230 与 MSPM0 联动 | 已完成基础闭环 | 自动追踪已实测，方向映射正确 |
| 阶段 4：云台方向与稳定性调参 | 进行中 | 当前问题是速度和运动连续性 |
| 阶段 5：文档和笔记 | 进行中 | 本文件为阶段交接记录 |

## 3. 当前硬件

- K230 视觉模块：亚博智能 K230 豪华版
- MSPM0 开发板：TI MSPM0G3507 地猛星开发板
- 云台：WHEELTEC 二维步进云台机械部分
- 步进驱动：WHEELTEC D36A 双路步进电机驱动板
- 目标物：当前实验使用橙色球
- 后续可能目标：蓝灰色球或 YOLO 检测目标

## 4. MSPM0 工程

工程目录：

```text
D:\01Project\EE\tracking_gimbal\tracking_gimbal
```

构建产物：

```text
Debug\tracking_gimbal.out
Debug\tracking_gimbal.hex
Debug\tracking_gimbal.map
```

当前 MSPM0 工程不依赖
`D:\01Project\EE\gimbal_learning\02_Gimbal_PosiCtrl`。

### 4.1 电机控制接线

| D36A | MSPM0G3507 | 功能 |
|---|---|---|
| ST1 | PA24 | X 轴 STEP |
| DIR1 | PA13 | X 轴 DIR |
| EN1 | PA16 | X 轴 EN |
| ST2 | PA22 | Y 轴 STEP |
| DIR2 | PA14 | Y 轴 DIR |
| EN2 | PA17 | Y 轴 EN |
| GND | GND | 必须共地 |

D36A 设置为 16 细分。

### 4.2 双 UART 串口接线

当前串口分工：

| 用途 | 外设 | MSPM0 TX | MSPM0 RX | 波特率 |
|---|---|---|---:|---:|
| K230 目标数据 | UART0 | PA28 | PA31 | 115200 |
| 电脑调试信息 | UART1 | PA26 | PA25 | 115200 |

两路串口的数据格式均为 8 数据位、1 停止位、无校验。

注意：SysConfig 中的 `UART_1` 实际分配给芯片的 `UART3` 外设，
这是配置名称与硬件外设编号不同造成的，不影响功能。

K230 官方串口线色：

| K230 线色 | K230 信号 |
|---|---|
| 红色 | 5V |
| 绿色 | TXD |
| 黄色 | RXD |
| 黑色 | GND |

K230 接线：

| K230 | MSPM0G3507 |
|---|---|
| TXD(IO9) | PA31，UART0 RX |
| RXD(IO10) | PA28，UART0 TX，可选 |
| GND | GND |
| 5V | 不接 |

实测必须优先看 IO9/IO10 丝印，不能只看线材颜色。此前多次通信失败
都是 IO9/IO10 接反造成的。

电脑调试接线：

| USB 转串口 | MSPM0G3507 |
|---|---|
| RXD | PA26，UART1 TX |
| TXD | PA25，UART1 RX，当前可选 |
| GND | GND |

注意：

- 红色 5V 不要接到 MSPM0。
- K230 和 MSPM0 必须共地。
- 不要把电脑 USB-TTL 的 TX 和 K230 TX 同时接到 PA31。
- UART1 已启用 RX 中断，用于接收电脑命令。
- CCS SWD 调试使用 PA19/PA20，不受上述串口影响。

## 5. MSPM0 软件当前完成内容

关键目录：

```text
tracking_gimbal\
  empty.c
  empty.syscfg
App\
    tracking.c
    tracking.h
    usart.c
    usart.h
    yb_protocol.c
    yb_protocol.h
  Hardware\
    board.c
    board.h
    control.c
    control.h
    key.c
    key.h
    led.c
    led.h
    motor.c
    motor.h
```

### 5.1 UART 和协议

- `SYSCFG_DL_init()` 根据 `empty.syscfg` 初始化 UART0 和 UART1。
- `Uart0_Init()` 打开 UART0 的 NVIC 接收中断。
- `UART_0_INST_IRQHandler()` 每收到一个字节调用
  `Pto_Data_Receive()`。
- `Debug_Uart_SendString()` 和 `Debug_Uart_SendChar()` 通过
  UART1/PA26 向电脑输出调试信息。
- `printf` 也已重定向到 UART1。
- `Pto_Loop()` 在主循环中解析完整帧。
- 当前协议长度为总字符数，包含 `$` 和 `#`。
- 当前支持功能 ID 为 `1` 的颜色/目标位置数据。

实际收到的测试帧：

```text
$23,01,020,030,040,050#
```

MSPM0 串口输出：

```text
YB target: x=20 y=30 w=40 h=50 cx=40 cy=55
```

### 5.2 当前 MSPM0 跟踪控制

已创建：

```text
App\tracking.c
App\tracking.h
```

当前行为：

- 上电后默认关闭自动跟踪，进入 UART1 命令测试模式。
- 支持 `A+`、`A-`、`B+`、`B-`、`STOP`、`TRACK`、`ZERO`、`RX?`、`POS?`。
- 手动测试每次移动 10 度。
- 摄像头按逆时针旋转 90 度映射：
  - A 轴使用图像 y 方向误差。
  - B 轴使用图像 x 方向误差。
- 自动跟踪使用比例速度模式，控制周期 40ms，最大速度 30 RPM。
- 每 40ms 最多改变 3 RPM，用于形成简单加减速。
- 软限位为正负 90 度，位置按实际发出的 STEP 脉冲累计。
- 目标丢失 200ms 后停止两轴。
- 单击按键停止跟踪，双击按键恢复。
- 自动跟踪可由 UART1 `TRACK` 命令或 K230 板载按键启动。
- K230 按键通过功能 ID 24 发送开始/停止控制帧。
- 本轮速度模式代码已编译通过，但尚未完成实机复测。

优化阶段的完整交接见：

```text
D:\01Project\EE\tracking_gimbal\OPTIMIZATION_HANDOFF.md
```

## 6. K230 脚本

### 6.1 当前使用的颜色识别脚本

文件：

```text
D:\01Project\EE\tracking_gimbal\k230_tracking_ball_color.py
```

主要流程：

```text
RGB888 图像
  -> RGB2HSV
  -> 橙色阈值掩码
  -> 开运算和闭运算
  -> 查找轮廓
  -> 面积、圆形度、长宽比筛选
  -> 最小外接圆
```

当前固定阈值：

```python
(0, 160, 140, 20, 255, 255)
```

含义：

```text
H: 0~20
S: 160~255
V: 140~255
```

最近一次有效自动校准结果：

```text
H=7.2 S=216.3 V=198.6
```

当前模式：

```python
TARGET_COLOR = "orange"
AUTO_CALIBRATE = False
DEBUG_SHOW_MASK = False
```

自动校准函数仍保留。更换灯光、背景或相机曝光后，可以把
`AUTO_CALIBRATE` 改为 `True` 重新校准一次。

### 6.2 阶段 3 UART 发送脚本

文件：

```text
D:\01Project\EE\tracking_gimbal\k230_tracking_ball_uart.py
```

使用官方库：

```python
from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
```

发送方式：

```python
uart = YbUart(baudrate=115200)
protocol = YbProtocol()

frame = protocol.get_color_data(left, top, width, height)
uart.send(frame)
```

发送策略：

```python
ENABLE_UART_SEND = True
UART_SEND_INTERVAL_MS = 40
```

即约 25 次/秒，目标真实丢失时不发送新帧。

该脚本已经完成真机联调，离线保存为 `main.py` 后会自动运行。

### 6.3 最小串口链路测试

文件：

```text
D:\01Project\EE\tracking_gimbal\k230_uart_link_test.py
```

该脚本不初始化摄像头，只通过官方 `YbUart` 和 `YbProtocol`
循环发送三组已知坐标，用于先排除颜色识别和球的干扰。

电脑调试串口应依次看到：

```text
YB target: x=20 y=30 w=40 h=50
YB target: x=140 y=110 w=40 h=40
YB target: x=250 y=180 w=30 h=30
```

三组数据会每 500ms 循环发送一次。链路测试通过后，停止该脚本，
再运行 `k230_tracking_ball_uart.py` 做真实视觉发送测试。

MSPM0 可发送 `RX?` 按需查询：

```text
UART0 RX bytes=原始字节数 frames=有效帧数
```

- `frames` 持续增加：K230 到 MSPM0 链路和协议都正常。
- `bytes` 不增加：K230 TX 没有连接好、接错 IO9/IO10 或没有共地。
- `bytes` 增加但 `frames=0`：没有形成完整合法帧。

## 7. 阶段 2 测试结果

日志目录：

```text
D:\01Project\EE\tracking_gimbal\test_data
```

最新有效日志：

```text
canmv-terminal-2026-10-04T05-57-48-911Z.log
```

统计：

```text
检测到球：915 帧
HOLD：19 帧
丢失：59 帧，包含启动和离场阶段
FPS：约 55
半径中位数：23 像素
圆形度中位数：0.73
```

用户实测结论：

```text
跟随稳定，响应及时
```

## 8. 当前关键参数

K230 识别参数：

```python
MIN_CONTOUR_AREA = 120
MIN_CIRCULARITY = 0.45
MAX_ASPECT_RATIO = 1.7
MIN_RADIUS = 6
MAX_RADIUS = 80
LOST_FRAME_LIMIT = 5
```

MSPM0 当前已写入代码的控制参数：

```text
FRAME_CENTER_X = 160
FRAME_CENTER_Y = 120
DEADZONE_A = 8
DEADZONE_B = 8
GAIN_FAST_A/B = 0.06
GAIN_SLOW_A/B = 0.02
MAX_ANGLE_STEP = 2.5
CONTROL_PERIOD_MS = 40
TRACKING_MOVE_SPEED_RPM = 35
SOFT_LIMIT_DEG = 30
TARGET_LOST_TIMEOUT_MS = 200
MANUAL_TEST_STEP_DEG = 10
```

## 9. 下一会话建议起点

阶段 3 已经完成基础联动，下一会话直接进入优化阶段：

1. 先读 `OPTIMIZATION_HANDOFF.md`。
2. 保持当前已经验证正确的 A/B 方向符号。
3. 保持 K230 `TXD(IO9) -> PA31`。
4. 优先把位置增量控制优化为连续速度控制。
5. 保留 `STOP`、按键停止、200ms 丢失停止和正负 30 度软限位。
6. 修改后先低速验证，再逐步调增益和最大 RPM。

## 10. 风险和注意事项

- UART0 已改为 K230 专用；电脑调试必须接 UART1，避免再次争用 PA31。
- 本次阶段 3 联调曾出现 `UART0 RX bytes=0`。最终确认 K230 与
  USB 转串口接在同一个 USB 拓展坞会造成通信异常；分开连接后恢复。
  后续排查 UART0 无数据时，应先排除共用拓展坞和供电干扰。
- 当前云台控制为开环步进控制，没有编码器、限位和自动归零。
- 多次快速调用 `Motor_Move_Relative()` 可能造成运动断续，联调时
  需要控制周期和速度限制。
- 橙色球颜色阈值依赖当前光照，换环境后应重新校准。
- 当前没有 Git 仓库，修改前建议手动备份关键文件。

## 11. 快速恢复命令

重新编译 MSPM0：

```powershell
cd D:\01Project\EE\tracking_gimbal\tracking_gimbal\Debug
& 'D:\Software\ti\ccs2050\ccs\utils\bin\gmake.exe' -B -f makefile all
```

检查 K230 Python 语法：

```powershell
cd D:\01Project\EE\tracking_gimbal
python -m py_compile k230_tracking_ball_color.py
python -m py_compile k230_tracking_ball_uart.py
```
