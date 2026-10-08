# 跟踪云台 MSPM0 工程

MSPM0G3507 固件、接线和控制协议说明。

## 主要功能

- 支持两轴开环位置控制，用于手动校准。
- `UART0` 专门接收 K230 数据，RX = `PA31`，TX = `PA28`，波特率 `115200`。
- `UART1` 专门连接电脑调试口，TX = `PA26`，RX = `PA25`，波特率 `115200`。
- 接收 K230 YbProtocol ASCII 帧，并通过 `UART1` 打印解析结果。
- 摄像头逆时针旋转 90 度后的 A/B 轴映射已写入 `tracking.c`。
- 上电后默认停止，通过 UART1 命令进入手动测试或自动跟踪。
- 支持命令：`A+`、`A-`、`B+`、`B-`、`STOP`、`TRACK`、`ZERO`、`RX?`、`POS?`。
- 电机方向：`A+` 右转、`A-` 左转、`B+` 抬头、`B-` 低头。
- 图像映射：`A+` 使 `cy` 减小，`B+` 使 `cx` 减小。
- 单击按键停止，双击按键恢复自动跟踪。
- K230 板载按键短按一次，可在开始跟踪和停止跟踪之间切换。
- 自动跟踪使用连续速度模式和 RPM 斜率限制，控制周期 40ms。
- 跟踪速度最大 30 RPM，每 40ms 最多变化 3 RPM。
- 相对上电位置设置正负 90 度软限位，位置按实际发出的 STEP 脉冲累计。
- 目标丢失超过 200ms 时停止两轴。

## 数据帧

```text
$23,01,020,030,040,050#
```

| 字段 | 值 | 含义 |
|---|---:|---|
| 帧头 | `$` | 一帧开始 |
| 长度 | `23` | 包含头尾的整帧字符数 |
| 功能 ID | `1` | 颜色或目标位置 |
| x | `20` | 目标左上角 X 坐标 |
| y | `30` | 目标左上角 Y 坐标 |
| w | `40` | 目标宽度 |
| h | `50` | 目标高度 |
| 帧尾 | `#` | 一帧结束 |

K230 板载按键控制帧：

```text
$09,24,1#
$09,24,2#
```

| 字段 | 值 | 含义 |
|---|---:|---|
| 长度 | `09` | 包含头尾的整帧字符数 |
| 功能 ID | `24` | 跟踪控制 |
| 动作 | `1` | 开始跟踪 |
| 动作 | `2` | 停止跟踪 |

## 串口分工

| 用途 | 外设 | MSPM0 TX | MSPM0 RX | 波特率 |
|---|---|---|---:|---:|
| K230 目标数据 | UART0 | PA28 | PA31 | 115200 |
| 电脑调试信息 | UART1 | PA26 | PA25 | 115200 |

UART1 实际使用芯片的 UART3 外设，这是 SysConfig 中的名称与硬件外设编号不同造成的，功能不受影响。

## K230 接线

| K230 信号 | MSPM0G3507 |
|---|---|
| K230 TXD(IO9) | PA31 UART0 RX |
| K230 RXD(IO10) | PA28 UART0 TX，可选 |
| K230 GND | GND |
| K230 5V | 不接 |

优先按接口丝印连接，不要只依赖线材颜色。

## 电脑调试接线

| USB 转串口 | MSPM0G3507 |
|---|---|
| RXD | PA26 UART1 TX |
| TXD | PA25 UART1 RX |
| GND | GND |

串口参数：

```text
115200
8 数据位
1 停止位
无校验
```

## 使用步骤

1. 在 CCS 中编译并烧录本工程。
2. 按“电脑调试接线”连接 USB 转串口。
3. 打开串口调试助手，选择 USB 转串口对应的端口，波特率设置为 `115200`。
4. 复位 MSPM0，应收到 `tracking_gimbal UART ready` 和
   `TRACK READY: A+/A-/B+/B-/STOP/TRACK/ZERO/POS?`。
5. 未连接 K230 时，先把云台放在安全中间位置进行手动校准。
6. 发送 `A+`，A 轴应转动约 10 度；再依次测试 `A-`、`B+`、`B-`。
7. 方向确认后发送 `ZERO`，把当前位置作为软件 0 度。
8. 自动跟踪前，保持 K230 已连接并持续发送有效目标。
9. 发送 `RX?` 可查询 `UART0 RX bytes=原始字节数 frames=有效帧数`。
10. 发送 `TRACK`，开始自动跟踪；发送 `STOP` 可立即停止。
11. 发送 `POS?`，可查看两轴软件 STEP 位置、当前 RPM 和限位步数。
12. 运行 `k230_tracking_ball_uart.py`。移动橙色球时，调试串口应持续输出变化的 `x/y/w/h`。
13. 短按 K230 板载按键，应看到 `YB control: START` 和 `TRACK START`；再次短按应看到 `YB control: STOP` 和 `TRACK STOP`。

## K230 离线运行

1. 用 CanMV IDE 打开 `k230_tracking_ball_uart.py`。
2. 使用“Save open script to CanMV board（as main.py）”保存到 K230。
3. 断开 K230 的 Type-C 数据线。
4. 重新给 K230 上电，脚本会自动运行，并通过 LCD 显示识别状态。

## 安全操作

- 发送 `STOP` 或单击 MSPM0 按键：立即停止自动跟踪和两轴运动。
- 发送 `TRACK` 或双击 MSPM0 按键：重新开始自动跟踪。
- K230 板载按键短按：切换自动跟踪开始/停止。
- K230 丢失目标：MSPM0 在 200ms 后停止两轴。
- 当前没有编码器、限位开关和自动回零，上电位置就是软件 0 度。
- 软限位使用的是已发 STEP 脉冲估计值，不是编码器真实角度。

## 目录结构

```text
App/
  tracking.c
  tracking.h
  usart.c
  usart.h
  yb_protocol.c
  yb_protocol.h
Hardware/
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
empty.c
empty.syscfg
```

## 软件数据流

```text
UART0 RX 中断，PA31 接收 K230
  -> Pto_Data_Receive()
  -> 收集完整帧
  -> New_CMD_flag
  -> Pto_Loop()
  -> Pto_Data_Parse()
  -> 目标帧：保存 x/y/w/h，并通过 UART1 输出解析结果
  -> 控制帧：设置开始/停止跟踪请求
  -> Tracking_Loop() 计算目标中心与画面中心误差
  -> 比例控制换算为目标 RPM
  -> 每 40ms 做一次速度斜率限制和软限位检查
  -> Motor_SetAxisSpeed_RPM() 连续输出 STEP
```

## 构建产物

```text
Debug/tracking_gimbal.out
Debug/tracking_gimbal.hex
Debug/tracking_gimbal.map
```
