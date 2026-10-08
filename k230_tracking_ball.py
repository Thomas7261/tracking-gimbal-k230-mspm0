# k230_tracking_ball.py - K230 蓝灰色球识别实验
#
# 当前阶段只验证视觉识别：
#   摄像头 -> 灰度图 -> 高斯模糊 -> 霍夫圆检测 -> 绘制结果 -> IDE 显示
#
# 本脚本暂不发送 UART 数据。识别稳定后，下一阶段再把圆心和外框坐标
# 通过 YbProtocol 发给 MSPM0G3507。
#
# ============================================================================
# 调参入口说明（先看这里）
# ============================================================================
#
# 推荐严格按照下面的顺序调参，一次只改一个变量：
#
#   第 1 步：确认球在画面里的半径 r 大约是多少。
#           看 IDE 打印 "ball cx=... cy=... r=..."。
#
#   第 2 步：先调 HOUGH_MIN_RADIUS 和 HOUGH_MAX_RADIUS。
#           它们决定“多大到多大的圆才允许被当成候选”。
#
#   第 3 步：再调 HOUGH_PARAM2。
#           它是控制误检和漏检最重要的一项。
#           误检多：增大；漏检多：减小。
#
#   第 4 步：调 HOUGH_MIN_DIST。
#           它决定两个候选圆心至少要隔多远。
#           同一个球出现两个圆：增大；多个目标太近：减小。
#
#   第 5 步：调 BLUR_KERNEL 和 BLUR_SIGMA。
#           背景边缘很多：加大模糊；圆轮廓被抹掉：减小模糊。
#
#   第 6 步：最后才考虑 HOUGH_DP 和 HOUGH_PARAM1。
#           这两个影响更细，通常不要和半径、param2 同时修改。
#
# ============================================================================

import os
import time
import gc
import sys

from media.sensor import *
from media.display import *
from media.media import *
import cv2


# ============================================================================
# 【调参位置 1】图像尺寸
# ============================================================================
#
# 直接使用 320x240，与 MSPM0 后续使用的坐标空间一致：
#   画面中心 = (160, 120)，不需要再做缩放。
#
# 一般不先改这里。降低分辨率可以提高 FPS，但圆会变小，半径范围也要跟着改。
DETECT_WIDTH = 320
DETECT_HEIGHT = 240
FRAME_CENTER_X = DETECT_WIDTH // 2
FRAME_CENTER_Y = DETECT_HEIGHT // 2

# 摄像头原始输出能力。最终仍会被 set_framesize() 缩放到检测分辨率。
# 当前 FPS 已经有 40，不需要通过降低这里来提速。
SENSOR_WIDTH = 1280
SENSOR_HEIGHT = 960
SENSOR_FPS = 90

# ============================================================================
# 【调参位置 2】霍夫圆参数
# ============================================================================
#
# 官方 OpenCV 霍夫圆例程使用 RGB888，本脚本也使用 RGB888。
# 不要单独把 set_pixformat 改成 RGB565，因为 RGB565 主要配合
# image.find_blobs() 颜色识别，而 OpenCV cv2.HoughCircles() 需要
# 当前这种 RGB888 numpy 图像。
#
# dp：
#   累加器分辨率与图像分辨率的比例。1 表示使用原分辨率。
#   当前 1.2 偏快；如果 FPS 足够且定位精度不够，可以尝试 1.0。
# minDist：
#   两个圆心之间允许的最小距离，用来避免同一个球被重复检测。
#   如果同一个球反复出现两个圆圈，把它调大。
# param1：
#   Canny 边缘检测的高阈值。
#   边缘太杂乱时轻微调大；球边缘太弱时轻微调小。
# param2：
#   圆心累加阈值。减小会增加误检，增大会漏检。
#   ★ 当前最优先调的参数 ★
# minRadius / maxRadius：
#   允许检测的球半径范围，单位是像素。
#   ★ 第二步优先调的参数 ★
#   数值必须根据真实打印的 r 来设置，不能照着示例值硬抄。
HOUGH_DP = 1.2
HOUGH_MIN_DIST = 35
HOUGH_PARAM1 = 100
HOUGH_PARAM2 = 38
HOUGH_MIN_RADIUS = 10
HOUGH_MAX_RADIUS = 65

# ============================================================================
# 【调参位置 3】高斯模糊
# ============================================================================
#
# 模糊核的大小必须为奇数，例如 3、5、7、9。
# 模糊太大：球的轮廓被抹平，容易漏检。
# 模糊太小：纹理和噪声边缘增多，容易误检。
#
# 计算公式（一维）：
#
#   G(x) = exp(-x^2 / (2 * sigma^2))
#
# 再让所有位置的 G(x) 总和等于 1，就得到归一化权重。
# 二维高斯模糊等价于先水平模糊一次，再竖直模糊一次，所以计算量比
# 直接套二维公式小很多。
#
# 核半径和尺寸的关系：
#
#   radius = (ksize - 1) / 2
#   ksize  = 2 * radius + 1
#
# 例如：
#   ksize=(5,5) -> radius=2，读取中心左右、上下各 2 个像素。
#   ksize=(7,7) -> radius=3，读取中心左右、上下各 3 个像素。
#
# sigma 决定权重从中心向外下降得多快：
#   sigma 越大：远处像素权重越大，边缘越模糊。
#   sigma 越小：权重越集中到中心，边缘越清楚。
#
# 经验范围：
#   radius 通常取 2*sigma 到 3*sigma。
#   r=15 的小球优先尝试 ksize=(5,5)、sigma=0.8~1.0。
#   原参数 ksize=(7,7)、sigma=1.5 时，中心 3x3 以外约占 50% 权重，
#   对小球的圆边缘影响很明显。
#
# 调参顺序：
#   1. 先保持 sigma 不变，减小 ksize，观察小圆是否重新出现。
#   2. 再保持 ksize 不变，逐步减小 sigma，例如 1.5 -> 1.0 -> 0.8。
#   3. param2 只有在 Canny 已经产生完整圆边缘后才有效。
BLUR_KERNEL = (5, 5)
BLUR_SIGMA = 1.0

sensor = None


def init_camera():
    """初始化摄像头并输出 320x240 RGB888 图像。"""
    global sensor

    sensor = Sensor(
        width=SENSOR_WIDTH,
        height=SENSOR_HEIGHT,
        fps=SENSOR_FPS,
    )
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.RGB888)
    sensor.run()
    time.sleep_ms(500)


def detect_ball(img_np):
    """
    在 BGR/RGB888 numpy 图像中寻找最可信的圆球。

    返回：
        (True, cx, cy, radius)  找到目标
        (False, 0, 0, 0)        没有找到目标
    """
    # 灰度图只保留亮度信息，降低计算量，也让圆边缘检测更稳定。
    # 注意：霍夫圆不理解“颜色”，它只理解灰度边缘。
    gray = cv2.cvtColor(img_np, cv2.COLOR_BGR2GRAY)

    # 高斯模糊抑制噪点和细小纹理，避免把杂乱边缘当成圆周。
    blurred = cv2.GaussianBlur(gray, BLUR_KERNEL, BLUR_SIGMA)

    # ★【调参位置 4】霍夫圆函数 ★
    # 这里的参数全部来自上面的调参区。
    circles = cv2.HoughCircles(
        blurred,
        3,
        dp=HOUGH_DP,
        minDist=HOUGH_MIN_DIST,
        param1=HOUGH_PARAM1,
        param2=HOUGH_PARAM2,
        minRadius=HOUGH_MIN_RADIUS,
        maxRadius=HOUGH_MAX_RADIUS,
    )

    if circles is None or circles.shape[0] == 0:
        return False, 0, 0, 0

    # 当前先采用霍夫变换返回的第一个圆。
    # 第一个候选通常来自投票最高的圆周。
    #
    # 这里也是“锁定到别的物体”的关键位置：
    # 霍夫结果只说明它像一个圆，不代表它就是目标球。
    # 下一版会在这里增加“上一帧位置优先”和颜色验证。
    cx = int(circles[0, 0])
    cy = int(circles[0, 1])
    radius = int(circles[0, 2])

    # 再次检查数值边界，防止异常候选进入后续控制逻辑。
    if radius < HOUGH_MIN_RADIUS or radius > HOUGH_MAX_RADIUS:
        return False, 0, 0, 0
    if cx < 0 or cx >= DETECT_WIDTH or cy < 0 or cy >= DETECT_HEIGHT:
        return False, 0, 0, 0

    return True, cx, cy, radius


def draw_result(img, found, cx, cy, radius, fps):
    """把检测结果和帧率画到 IDE 预览图像上。"""
    # 先画画面中心参考线，方便观察球是否接近中心。
    img.draw_cross(
        FRAME_CENTER_X,
        FRAME_CENTER_Y,
        size=8,
        thickness=1,
        color=(255, 255, 255),
    )

    if found:
        left = cx - radius
        top = cy - radius
        width = radius * 2
        height = radius * 2

        # 红色外圈表示检测到的球。
        img.draw_circle(
            cx,
            cy,
            radius,
            color=(255, 0, 0),
            thickness=2,
        )

        # 绿色十字表示球心。
        img.draw_cross(
            cx,
            cy,
            size=6,
            thickness=2,
            color=(0, 255, 0),
        )

        # 黄色矩形是准备发给 MSPM0 的 x/y/w/h 外框。
        img.draw_rectangle(
            left,
            top,
            width,
            height,
            color=(255, 255, 0),
            thickness=1,
        )

        text = "ball x=%d y=%d r=%d" % (cx, cy, radius)
        img.draw_string_advanced(
            4,
            4,
            16,
            text,
            color=(0, 255, 0),
        )
    else:
        img.draw_string_advanced(
            4,
            4,
            18,
            "TARGET LOST",
            color=(255, 0, 0),
        )

    img.draw_string_advanced(
        4,
        28,
        16,
        "FPS %.1f" % fps,
        color=(255, 255, 255),
    )


def main():
    global sensor

    init_camera()

    # ST7701 主要给 IDE 预览使用。
    # 即使检测图像是 320x240，显示画布也可以使用更大的尺寸。
    Display.init(
        Display.ST7701,
        width=640,
        height=480,
        to_ide=True,
    )

    clock = time.clock()
    frame_count = 0
    fps = 0.0

    try:
        while True:
            os.exitpoint()
            clock.tick()

            img = sensor.snapshot()
            img_np = img.to_numpy_ref()

            found, cx, cy, radius = detect_ball(img_np)
            fps = clock.fps()

            draw_result(img, found, cx, cy, radius, fps)
            Display.show_image(img)

            frame_count += 1
            if frame_count >= 10:
                gc.collect()
                frame_count = 0

            if found:
                print(
                    "FPS %.1f  ball cx=%d cy=%d r=%d x=%d y=%d w=%d h=%d"
                    % (
                        fps,
                        cx,
                        cy,
                        radius,
                        cx - radius,
                        cy - radius,
                        radius * 2,
                        radius * 2,
                    )
                )
            else:
                print("FPS %.1f  target lost" % fps)

    except KeyboardInterrupt as exc:
        print("user stop:", exc)
    except BaseException as exc:
        sys.print_exception(exc)
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)


if __name__ == "__main__":
    main()
