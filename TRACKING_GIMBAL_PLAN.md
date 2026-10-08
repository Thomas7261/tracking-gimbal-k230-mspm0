# K230 + MSPM0G3507 跟踪云台实施方案

## 1. 文档用途

本文档是“方案研讨”阶段的最终产物，供后续 agent 直接执行。

当前硬件：

- 亚博智能 K230 豪华版
- WHEELTEC 二维步进云台机械部分
- WHEELTEC D36A 双路步进电机驱动板
- TI MSPM0G3507 地猛星开发板
- CCS 开发环境
- 目标物：蓝灰色、网球尺寸球体

当前没有 WHEELTEC C12H 主控板，因此系统采用：

```text
K230 摄像头
   |
   | UART
   v
MSPM0G3507
   |
   | STEP / DIR / EN
   v
D36A 双路步进驱动板
   |
   v
X 轴 / Y 轴 42 步进电机
```

## 2. 第一版目标

第一版只做稳定可复现的“基础跟踪”：

1. K230 识别蓝灰色球体。
2. K230 计算球心在画面中的位置。
3. K230 通过串口把目标位置发给 MSPM0。
4. MSPM0 根据目标偏离画面中心的距离，控制 X/Y 两轴转动。
5. 目标丢失时云台停止，不继续乱转。

第一版不做：

- 编码器闭环控制。
- YOLO 自训练模型。
- 高速目标跟踪。
- 自动归零、机械限位保护。
- 复杂 PID 调参。

## 3. 验收标准

以下条件全部满足，第一版才算完成：

- 球放在摄像头前方，初始偏离中心不超过 30 度时，云台能在 2 秒内把球带回画面中心附近。
- 球进入中心死区后，云台不再持续抖动。
- 目标被遮挡或移出视野后，MSPM0 在 200ms 内停止两个轴。
- K230 检测帧率至少达到 10 FPS。
- 串口接收稳定，连续运行 5 分钟不出现解析错误。
- 蓝灰色球在普通室内光照和比赛常见背景中可以稳定识别。

## 4. 最终推荐方案

### 4.1 主控选择

使用 MSPM0G3507，开发环境使用 CCS。

理由：

- 你已经有 `gimbal_learning` 中的位置控制和速度控制学习基础。
- WHEELTEC 提供了 MSPM0G3507 的 CCS 工程。
- 云台驱动所需的 STEP/DIR/EN 引脚和代码逻辑已经验证过。

### 4.2 云台控制基础

复用 `02_Gimbal_PosiCtrl` 作为运动底座。

该工程已经实现：

- 角度转脉冲数：`steps = angle / 360 * 3200`
- 16 细分：每圈 3200 个脉冲
- 双轴 STEP 输出
- DIR 方向输出
- EN 使能保持
- 到位后停止脉冲并保持锁定

第一版不使用编码器反馈，属于开环位置控制。

### 4.3 识别方式

第一版使用“霍夫圆检测”为主，必要时增加蓝灰色掩膜。

原因：

- 蓝灰色球颜色饱和度低，纯颜色阈值容易受背景影响。
- 目标是圆球，圆轮廓是稳定特征。
- K230 资料中已有 `cv2.HoughCircles` 和 `cv_lite` 圆检测例程。

后续如果要换目标物体，只需要替换 K230 的检测模块，输出仍然保持统一的 `TargetObservation`，MSPM0 不需要修改。

### 4.4 通信方案

使用 UART，波特率 `115200`。

第一版复用亚博智能已有的 YbProtocol ASCII 帧：

```text
$<总长度>,<功能ID>,<x>,<y>,<w>,<h>#
```

示例含义：

- `x`：目标框左上角 X 坐标
- `y`：目标框左上角 Y 坐标
- `w`：目标框宽度
- `h`：目标框高度

K230 发送的是“检测结果”，不是“识别算法细节”。因此以后无论改成霍夫圆、颜色识别、YOLO、AprilTag 还是 NanoTracker，只要最终生成 `x,y,w,h`，MSPM0 都能工作。

这个方案称为“目标观察协议”。它是本项目最通用的接口。

### 4.5 第一版帧率

K230 建议使用 `320x240` 图像分辨率发送目标坐标。

如果 K230 使用 `640x480` 识别，发送前将坐标和宽高缩放到 `320x240` 空间。这样 MSPM0 端的画面中心常量只需要维护一份。

## 5. 硬件接线

### 5.1 D36A 到 MSPM0G3507

| D36A | MSPM0G3507 | 说明 |
|---|---|---|
| ST1 | PA24 | X 轴脉冲 |
| DIR1 | PA13 | X 轴方向 |
| EN1 | PA16 | X 轴使能 |
| ST2 | PA22 | Y 轴脉冲 |
| DIR2 | PA14 | Y 轴方向 |
| EN2 | PA17 | Y 轴使能 |
| GND | GND | 必须共地 |

注意：

- D36A 必须设置为 16 细分。
- D36A 与 MSPM0 可以分别供电，但必须共地。
- 电机电源按电机和 D36A 要求供电。

### 5.2 K230 到 MSPM0G3507

推荐使用 MSPM0 的 UART0：

| K230 | MSPM0G3507 | 说明 |
|---|---|---|
| TX | PA31 | K230 发送到 MSPM0 接收 |
| RX | PA28 | MSPM0 发送到 K230 接收 |
| GND | GND | 必须共地 |

接线原则：

```text
K230 TX -> MSPM0 RX
K230 RX -> MSPM0 TX
K230 GND -> MSPM0 GND
```

不要使用 PA22 作为 K230 通信引脚，因为 PA22 已经用于 Y 轴 STEP。

### 5.3 调试接口

| 信号 | MSPM0 引脚 |
|---|---|
| SWDIO | PA19 |
| SWCLK | PA20 |
| USB 转串口 TX | PA25，UART1 RX |
| USB 转串口 RX | PA26，UART1 TX |

## 6. MSPM0 软件结构

在 `02_Gimbal_PosiCtrl` 基础上新增三层：

```text
empty.c
   |
   +--> UART 接收中断
   |       |
   |       +--> Pto_Data_Receive()
   |
   +--> tracking.c / tracking.h
           |
           +--> Tracking_Update(x, y, w, h)
           |       |
           |       +--> 计算误差
           |       +--> 计算两轴角度增量
           |       +--> Motor_Move_Relative()
           |
           +--> Tracking_TimeoutCheck()
                   |
                   +--> 目标丢失则停止两轴
```

推荐目录结构：

```text
04_Gimbal_Tracking/
  empty.c
  empty.h
  empty.syscfg
  Hardware/
    board.c
    board.h
    control.c
    control.h
    motor.c
    motor.h
  App/
    usart.c
    usart.h
    yb_protocol.c
    yb_protocol.h
    tracking.c
    tracking.h
```

### 6.1 UART 接收

可以直接参考 K230 资料中的：

```text
MSPM0-K230/MSPM0G3507/01_k230_color_detect/APP/usart.c
MSPM0-K230/MSPM0G3507/01_k230_color_detect/APP/yb_protocol.c
```

需要修改：

- UART0 使用 PA31/PA28 连接 K230，UART1 使用 PA25/PA26
  连接电脑调试口，避免 PA22 冲突。
- 波特率 `115200`。
- 开启 RX 中断。
- 接收完成后设置 `New_CMD_flag`。

### 6.2 视觉伺服逻辑

MSPM0 收到一帧后：

```text
cx = x + w / 2
cy = y + h / 2

error_x = FRAME_CENTER_X - cx
error_y = FRAME_CENTER_Y - cy
```

其中：

```text
FRAME_CENTER_X = 160
FRAME_CENTER_Y = 120
```

控制规则：

1. 如果 `error_x` 和 `error_y` 都在死区内，不发送新移动命令。
2. 如果误差较大，使用较大增益。
3. 如果误差较小，使用较小增益。
4. 单次角度增量必须限幅，避免云台冲过头。
5. 每隔固定控制周期才发送一次新角度命令，不能每个串口帧都打断电机。

建议初始参数：

```text
DEADZONE_X = 8 像素
DEADZONE_Y = 8 像素
GAIN_FAST = 0.06 度/像素
GAIN_SLOW = 0.02 度/像素
MAX_ANGLE_STEP = 2.5 度
CONTROL_PERIOD_MS = 40
TRACKING_MOVE_SPEED_RPM = 35
```

这些参数只是起点，必须实际调试后确定。

### 6.3 目标丢失保护

如果超过 `TARGET_LOST_TIMEOUT_MS = 200` 没有收到有效帧：

- 调用 `Motor_StopAxis(GIMBAL_AXIS_X)`。
- 调用 `Motor_StopAxis(GIMBAL_AXIS_Y)`。
- 清空目标状态。

## 7. K230 软件结构

第一版脚本建议命名为：

```text
k230_tracking_ball.py
```

主流程：

```text
初始化摄像头
初始化显示
初始化 UART

while True:
    拍摄一帧
    对图像做降采样、灰度、模糊
    霍夫圆检测
    从候选圆中筛选最可信目标

    如果找到目标：
        绘制圆和中心十字
        把 x,y,w,h 缩放到 320x240 空间
        发送 YbProtocol 帧
    如果没找到目标：
        不发送有效帧
        在画面上显示“目标丢失”
```

识别模块可以单独写成函数：

```python
def detect_ball(img):
    # 输入图像
    # 返回 (found, cx, cy, radius)
    ...
```

这样以后更换检测算法时，只需要替换 `detect_ball` 内部逻辑，串口输出保持不变。

### 7.1 蓝灰色球识别的调试重点

蓝灰色球容易受以下因素影响：

- 白色背景反光。
- 金属地面或浅色桌面。
- 阴影导致灰度变化。
- 球和背景的轮廓对比不足。

推荐先做：

1. 固定摄像头高度和距离。
2. 固定球的大致出现距离。
3. 在灰度和模糊图像上检测圆形。
4. 设置 `minRadius` 和 `maxRadius`，排除过小和过大的圆。
5. 如果背景圆干扰严重，增加颜色掩膜作为预处理。

## 8. 分阶段执行计划

### 阶段 0：环境与硬件确认

目标：确认 CCS 可以编译现有 MSPM0 工程，云台机械部分可以上电。

任务：

- 确认 CCS 已安装并导入 `gimbal_learning`。
- 确认 D36A 设置为 16 细分。
- 按接线表连接 D36A 和 MSPM0G3507。
- 给 D36A 和 MSPM0 共地。
- 编译并烧录 `02_Gimbal_PosiCtrl`。
- 使用按键确认 X/Y 轴可以正反转。

验收：

- CCS 编译 0 error。
- 两个轴都能转动。
- 停止后电机保持锁定。

### 阶段 1：建立 MSPM0 UART 接收能力

目标：MSPM0 能收到 K230 或电脑串口发送的数据。

任务：

- 复制 `02_Gimbal_PosiCtrl` 为 `04_Gimbal_Tracking`。
- 添加 `App/usart.c/h`。
- 添加 `App/yb_protocol.c/h`。
- 在 SysConfig 中配置 UART0 到 PA31/PA28，UART1 到 PA25/PA26。
- 在 `empty.c` 中初始化 UART 和中断。
- 收到 YbProtocol 帧后通过串口打印 `x,y,w,h`。

验收：

- 电脑串口发送 `$16,1,20,30,40,50#` 时，MSPM0 能打印解析结果。

### 阶段 2：K230 单独识别球

目标：不接云台，K230 能在画面中稳定识别蓝灰色球。

任务：

- 基于 K230 的 OpenCV 霍夫圆例程编写 `k230_tracking_ball.py`。
- 绘制检测到的圆和中心点。
- 在 IDE 中打印 FPS、圆心和半径。
- 调整模糊、圆检测阈值和半径范围。

验收：

- 普通室内光照下，网球尺寸球能被连续检测。
- FPS 大于等于 10。
- 画面中没有过多误检圆。

### 阶段 3：K230 与 MSPM0 联调

目标：K230 发送目标位置，MSPM0 能解析并做出运动响应。

任务：

- 连接 K230 TX/RX/GND 到 MSPM0 UART0。
- 在 MSPM0 中加入 `tracking.c/h`。
- 将解析出的 `x,y,w,h` 转为视觉伺服误差。
- 调用 `Motor_Move_Relative` 控制 X/Y。
- 增加目标丢失超时保护。

验收：

- 球向左偏移时，云台向对应方向转动。
- 球到达中心附近后，云台停止。
- 球移出画面后，云台停止。

### 阶段 4：方向、增益和稳定性调试

目标：让跟踪动作平滑、不震荡、不过冲。

任务：

- 校准 X/Y 方向符号。
- 调整死区。
- 调整快慢增益。
- 调整最大单步角度。
- 调整控制周期。
- 调整目标丢失超时。
- 记录不同距离下的表现。

验收：

- 满足第 3 节验收标准。
- 连续运行 5 分钟无异常。

### 阶段 5：文档与笔记回写

任务：

- 写 README。
- 整理接线表。
- 记录调参结果。
- 更新 Obsidian 学习笔记。
- 标记后续可以升级的模块。

## 9. 关键文件来源

### 云台学习工程

```text
D:\01Project\EE\gimbal_learning\02_Gimbal_PosiCtrl
```

### MSPM0 UART/YbProtocol 参考

```text
D:\03university\比赛\电子设计\K230视觉模块\程序源码\14.export\MSPM0-K230\MSPM0G3507\01_k230_color_detect
```

### K230 颜色识别参考

```text
D:\03university\比赛\电子设计\K230视觉模块\程序源码\14.export\CanmvIDE-K230\01.color_detect.py
```

### K230 霍夫圆检测参考

```text
D:\03university\比赛\电子设计\K230视觉模块\程序源码\06.2 OpenCV\05.camera_hough_circles.py
D:\03university\比赛\电子设计\K230视觉模块\程序源码\06.1 cv_lite\4.rgb888_find_circles.py
```

### 云台接线说明

```text
D:\03university\比赛\电子设计\WHEELTEC 步进电机二维云台附赠资料包\2.源码\Ti MSPM0G3507例程\接线和使用说明.txt
```

## 10. 风险与应对

### 10.1 蓝灰色球识别不稳定

应对：

- 先用固定背景测试。
- 增加颜色掩膜。
- 用 NanoTracker 或训练模型作为后续升级。

### 10.2 开环步进电机丢步

应对：

- 降低移动速度。
- 限制单次角度增量。
- 检查 D36A 电流和 16 细分设置。
- 后续接入编码器做闭环。

### 10.3 UART 引脚冲突

应对：

- 固定使用 PA31/PA28 作为 K230 通信，PA25/PA26 作为电脑调试口。
- 不要使用 PA22、PA24、PA13、PA14、PA16、PA17 作为 UART 功能。

### 10.4 云台震荡

应对：

- 增大死区。
- 降低近中心增益。
- 降低最大单步角度。
- 降低控制频率。

### 10.5 机械限位

第一版没有限位开关和自动归零，调试时使用小角度范围。后续可增加限位或机械挡块。

## 11. 后续升级方向

这些内容不在第一版范围，但保留接口：

- 使用带校验的二进制协议替换 ASCII 协议。
- MSPM0 做绝对角度控制。
- MSPM0 做 PD/PID 控制。
- 接入编码器做闭环。
- K230 使用 NanoTracker。
- K230 使用 YOLO 目标检测。
- 使用 AprilTag 提高稳定性。
- 增加限位开关、归零和急停。

## 12. 后续 agent 的交付物

后续执行 agent 应交付：

1. CCS 工程 `04_Gimbal_Tracking`。
2. K230 脚本 `k230_tracking_ball.py`。
3. 项目 README。
4. 接线图和引脚表。
5. 调参记录。
6. 已更新的 Obsidian 学习笔记。
