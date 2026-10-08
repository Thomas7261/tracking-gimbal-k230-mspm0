# k230_tracking_ball_uart.py - K230 彩色球识别与 UART 上报
#
# 处理流程：
#   摄像头 -> HSV 颜色分割 -> 形态学去噪 -> 轮廓筛选
#   -> 最小外接圆 -> YbProtocol 数据帧 -> YbUart 发送
#
# 使用 YbUart 和 YbProtocol：
#   YbUart 负责 K230 板载串口的物理收发。
#   YbProtocol 负责生成 $len,1,x,y,w,h# 数据帧。
#   板载按键通过功能 ID 24 发送 TRACK/STOP 控制命令。
#   目标丢失时不发送新帧，由 MSPM0 的超时保护停止云台。

import os
import time
import gc
import sys
import math

from media.sensor import *
from media.display import *
from media.media import *
import cv2
import ulab.numpy as np
from libs.YbProtocol import YbProtocol
from ybUtils.YbKey import YbKey
from ybUtils.YbUart import YbUart


# 目标颜色
#
# 每个颜色包含一组或多组 HSV 范围：
#   [Hmin, Smin, Vmin, Hmax, Smax, Vmax]
#
# H 表示色调，S 表示饱和度，V 表示明度。
# 红色在 HSV 色环的 0 和 180 两端，所以需要两组范围。
TARGET_COLOR = "orange"

COLOR_PRESETS = {
    "red": [
        (0, 80, 60, 10, 255, 255),
        (170, 80, 60, 180, 255, 255),
    ],
    "orange": [
        # 当前照明条件下使用的橙色球 HSV 范围。
        (0, 160, 140, 20, 255, 255),
    ],
    "yellow": [
        (20, 80, 60, 35, 255, 255),
    ],
    "green": [
        (35, 70, 50, 85, 255, 255),
    ],
    "blue": [
        (90, 70, 50, 130, 255, 255),
    ],
}

HSV_RANGES = COLOR_PRESETS[TARGET_COLOR]

# 自动颜色校准
#
# 开启后从画面中心区域采样，并根据平均 HSV 自动生成颜色阈值。
AUTO_CALIBRATE = False
CALIBRATION_FRAMES = 30
CALIBRATION_ROI_HALF = 20

# 自动阈值允许偏离平均值的范围。
CAL_H_TOLERANCE = 10
CAL_S_MARGIN = 45
CAL_V_MARGIN = 45


# 图像尺寸和形态学
#
# 检测分辨率与 MSPM0 使用的目标坐标空间一致。
DETECT_WIDTH = 320
DETECT_HEIGHT = 240
FRAME_CENTER_X = DETECT_WIDTH // 2
FRAME_CENTER_Y = DETECT_HEIGHT // 2

# 识别仍然使用 320x240，只把显示画面放大到 LCD 分辨率。
# PREVIEW_SCALE_ENABLE:
#   True  = 放大显示，方便观察；
#   False = 保持原始 320x240 小画面，最省算力。
# PREVIEW_INTERPOLATION:
#   "linear"  = 平滑放大，推荐默认使用；
#   "nearest" = 最近邻放大，速度更快，但边缘会更像素化。
PREVIEW_SCALE_ENABLE = True
PREVIEW_INTERPOLATION = "linear"
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480

SENSOR_WIDTH = 1280
SENSOR_HEIGHT = 960
SENSOR_FPS = 90

# 开运算先去掉孤立噪点，闭运算再填补颜色区域内部的小孔。
MORPH_KERNEL_SIZE = (5, 5)

# 为 True 时显示 HSV 颜色掩码，用于检查检测范围。
DEBUG_SHOW_MASK = False


# UART 上报
#
# UART_SEND_INTERVAL_MS = 40 表示最快约 25 次/秒。
# 这样既足够跟踪，又不会让 MSPM0 的接收和解析任务持续满载。
ENABLE_UART_SEND = True
UART_SEND_INTERVAL_MS = 40

# 板载按键：短按一次切换跟踪开始/停止。
ENABLE_KEY_TRACKING = True
KEY_DEBOUNCE_MS = 250
KEY_CONTROL_FUNC = 24
KEY_CONTROL_START = 1
KEY_CONTROL_STOP = 2


# 轮廓和圆形筛选
#
# 最小轮廓面积。
MIN_CONTOUR_AREA = 120

# 最小圆形度，标准圆接近 1.0。
MIN_CIRCULARITY = 0.45

# 外接矩形长边和短边的最大比例。
MAX_ASPECT_RATIO = 1.7

# 最小外接圆允许的半径范围。
MIN_RADIUS = 6
MAX_RADIUS = 80

# 连续丢失多少帧后，认为目标真正丢失。
# 这段时间内会保留最后一次目标位置，避免一两帧漏检导致坐标跳空。
LOST_FRAME_LIMIT = 5


def init_camera():
    """初始化摄像头并输出 320x240 RGB888 图像。"""
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
    return sensor


def build_color_mask(hsv):
    """
    根据 HSV 范围生成颜色掩码。

    参数：
        hsv: cv2.cvtColor 转换后的 HSV numpy 图像

    返回：
        单通道掩码，目标颜色像素为 255，其余像素为 0。
    """
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        MORPH_KERNEL_SIZE,
    )

    mask = None
    for h_min, s_min, v_min, h_max, s_max, v_max in HSV_RANGES:
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        part = cv2.inRange(hsv, lower, upper)

        if mask is None:
            mask = part
        else:
            # 红色有两段 H 范围，需要把两个掩码合并。
            mask = cv2.bitwise_or(mask, part)

    # 开运算去噪，闭运算填补目标内部的小孔。
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def calibrate_color(sensor):
    """
    在画面中央测量橙色球的平均 HSV，并生成颜色阈值。

    返回：
        (h_mean, s_mean, v_mean)
    """
    global HSV_RANGES

    center_x = DETECT_WIDTH // 2
    center_y = DETECT_HEIGHT // 2
    left = center_x - CALIBRATION_ROI_HALF
    top = center_y - CALIBRATION_ROI_HALF
    right = center_x + CALIBRATION_ROI_HALF
    bottom = center_y + CALIBRATION_ROI_HALF

    sum_h = 0.0
    sum_s = 0.0
    sum_v = 0.0

    print("AUTO CALIBRATION: put the ball in the center")

    for index in range(CALIBRATION_FRAMES):
        img = sensor.snapshot()
        img_np = img.to_numpy_ref()
        # Sensor.RGB888 的输出按 RGB 通道顺序转换，避免橙色被误算成蓝色。
        hsv = cv2.cvtColor(img_np, cv2.COLOR_RGB2HSV)
        center_roi = hsv[top:bottom, left:right]
        mean_hsv = cv2.mean(center_roi)

        sum_h += mean_hsv[0]
        sum_s += mean_hsv[1]
        sum_v += mean_hsv[2]

        img.draw_rectangle(
            left,
            top,
            CALIBRATION_ROI_HALF * 2,
            CALIBRATION_ROI_HALF * 2,
            color=(255, 255, 0),
            thickness=2,
        )
        img.draw_string_advanced(
            4,
            4,
            18,
            "CALIBRATING %d/%d"
            % (index + 1, CALIBRATION_FRAMES),
            color=(255, 0, 0),
        )
        Display.show_image(img)
        time.sleep_ms(20)

    h_mean = sum_h / CALIBRATION_FRAMES
    s_mean = sum_s / CALIBRATION_FRAMES
    v_mean = sum_v / CALIBRATION_FRAMES

    h_min = max(0, int(h_mean - CAL_H_TOLERANCE))
    h_max = min(180, int(h_mean + CAL_H_TOLERANCE))
    s_min = max(40, int(s_mean - CAL_S_MARGIN))
    v_min = max(30, int(v_mean - CAL_V_MARGIN))

    HSV_RANGES = [
        (h_min, s_min, v_min, h_max, 255, 255),
    ]

    print(
        "CAL HSV mean: H=%.1f S=%.1f V=%.1f"
        % (h_mean, s_mean, v_mean)
    )
    print(
        "CAL HSV range: H=%d..%d S=%d..255 V=%d..255"
        % (h_min, h_max, s_min, v_min)
    )

    return h_mean, s_mean, v_mean


def detect_ball(img_np):
    """
    从 RGB888 图像中寻找最可信的彩色球。

    返回：
        (True, cx, cy, radius, area, circularity, mask)
        (False, 0, 0, 0, 0, 0.0, mask)
    """
    # Sensor.RGB888 的输出按 RGB 通道顺序转换。
    hsv = cv2.cvtColor(img_np, cv2.COLOR_RGB2HSV)
    mask = build_color_mask(hsv)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    best_candidate = None
    best_score = -1.0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_CONTOUR_AREA:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0.0:
            continue

        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < MIN_CIRCULARITY:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            continue

        long_side = max(w, h)
        short_side = min(w, h)
        aspect_ratio = float(long_side) / float(short_side)
        if aspect_ratio > MAX_ASPECT_RATIO:
            continue

        (cx_float, cy_float), radius_float = cv2.minEnclosingCircle(contour)
        cx = int(cx_float)
        cy = int(cy_float)
        radius = int(radius_float)

        if radius < MIN_RADIUS or radius > MAX_RADIUS:
            continue
        if cx < 0 or cx >= DETECT_WIDTH:
            continue
        if cy < 0 or cy >= DETECT_HEIGHT:
            continue

        # 面积越大、轮廓越接近圆，得分越高。
        score = area * circularity

        if score > best_score:
            best_score = score
            best_candidate = (
                cx,
                cy,
                radius,
                int(area),
                circularity,
            )

    if best_candidate is None:
        return False, 0, 0, 0, 0, 0.0, mask

    cx, cy, radius, area, circularity = best_candidate
    return True, cx, cy, radius, area, circularity, mask


def draw_result(
    img,
    found,
    cx,
    cy,
    radius,
    area,
    circularity,
    fps,
    tracking_enabled,
):
    """把颜色识别结果画到 IDE 预览中。"""
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

        img.draw_circle(
            cx,
            cy,
            radius,
            color=(0, 255, 0),
            thickness=2,
        )
        img.draw_cross(
            cx,
            cy,
            size=6,
            thickness=2,
            color=(255, 0, 0),
        )
        img.draw_rectangle(
            left,
            top,
            width,
            height,
            color=(255, 255, 0),
            thickness=1,
        )
        img.draw_string_advanced(
            4,
            4,
            16,
            "ball cx=%d cy=%d r=%d" % (cx, cy, radius),
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
    img.draw_string_advanced(
        4,
        52,
        16,
        "TRACK %s" % ("ON" if tracking_enabled else "OFF"),
        color=(0, 255, 255) if tracking_enabled else (255, 255, 255),
    )


def show_preview(image_data, pixel_format):
    """按配置把识别图像直接显示或放大后送到 LCD。"""
    if PREVIEW_SCALE_ENABLE:
        if PREVIEW_INTERPOLATION == "nearest":
            interpolation = cv2.INTER_NEAREST
        else:
            interpolation = cv2.INTER_LINEAR

        preview_np = cv2.resize(
            image_data,
            (DISPLAY_WIDTH, DISPLAY_HEIGHT),
            interpolation=interpolation,
        )
        preview_width = DISPLAY_WIDTH
        preview_height = DISPLAY_HEIGHT
    else:
        preview_np = image_data
        preview_width = DETECT_WIDTH
        preview_height = DETECT_HEIGHT

    preview_img = image.Image(
        preview_width,
        preview_height,
        pixel_format,
        alloc=image.ALLOC_REF,
        data=preview_np,
    )
    Display.show_image(preview_img)


def main():
    sensor = None
    lost_frames = 0
    last_observation = None
    uart = None
    protocol = None
    key = None
    last_send_ms = 0
    uart_send_count = 0
    tracking_enabled = False
    key_was_pressed = False
    last_key_ms = 0

    try:
        if ENABLE_UART_SEND:
            # 官方库使用 K230 板载串口，默认波特率 115200。
            uart = YbUart(baudrate=115200)
            protocol = YbProtocol()
            last_send_ms = time.ticks_ms()
            if ENABLE_KEY_TRACKING:
                key = YbKey()

        sensor = init_camera()
        Display.init(
            Display.ST7701,
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            to_ide=True,
        )

        calibration = None
        if AUTO_CALIBRATE:
            calibration = calibrate_color(sensor)

        clock = time.clock()
        frame_count = 0
        fps = 0.0

        while True:
            os.exitpoint()
            clock.tick()
            is_holding = False

            img = sensor.snapshot()
            img_np = img.to_numpy_ref()

            (
                found,
                cx,
                cy,
                radius,
                area,
                circularity,
                mask,
            ) = detect_ball(img_np)
            fps = clock.fps()

            # 按键只在“按下沿”触发一次，避免按住时连续发送。
            # 这里切换的是 K230 自己的跟踪状态，控制帧才是命令 MSPM0 的入口。
            if key is not None:
                key_pressed = key.is_pressed()
                key_now_ms = time.ticks_ms()

                if (
                    key_pressed
                    and not key_was_pressed
                    and time.ticks_diff(key_now_ms, last_key_ms)
                    >= KEY_DEBOUNCE_MS
                ):
                    tracking_enabled = not tracking_enabled
                    action = (
                        KEY_CONTROL_START
                        if tracking_enabled
                        else KEY_CONTROL_STOP
                    )
                    control_frame = protocol.package_message(
                        KEY_CONTROL_FUNC,
                        str(action),
                    )
                    uart.send(control_frame)
                    last_key_ms = key_now_ms
                    print(
                        "KEY TRACK:",
                        "ON" if tracking_enabled else "OFF",
                        control_frame,
                    )

                key_was_pressed = key_pressed

            if found:
                lost_frames = 0
                last_observation = (cx, cy, radius, area, circularity)
            else:
                lost_frames += 1

                # 短暂丢失时保持最后位置，方便观察目标为什么会中断。
                if (lost_frames <= LOST_FRAME_LIMIT) and (
                    last_observation is not None
                ):
                    cx, cy, radius, area, circularity = last_observation
                    found = True
                    is_holding = True
                else:
                    last_observation = None

            if found:
                if is_holding:
                    print(
                        "FPS %.1f  HOLD cx=%d cy=%d r=%d lost=%d"
                        % (fps, cx, cy, radius, lost_frames)
                    )
                else:
                    print(
                        "FPS %.1f  ball cx=%d cy=%d r=%d area=%d circ=%.2f"
                        % (fps, cx, cy, radius, area, circularity)
                    )
            else:
                print("FPS %.1f  target lost" % fps)

            # 只有真实检测到的帧才发送；HOLD 只是视觉显示用的旧坐标。
            # 目标丢失时不发送，MSPM0 超过 200ms 未收到数据即停止。
            if ENABLE_UART_SEND and found and not is_holding:
                now_ms = time.ticks_ms()
                if time.ticks_diff(now_ms, last_send_ms) >= (
                    UART_SEND_INTERVAL_MS
                ):
                    left = max(0, cx - radius)
                    top = max(0, cy - radius)
                    right = min(DETECT_WIDTH, cx + radius)
                    bottom = min(DETECT_HEIGHT, cy + radius)
                    width = max(0, right - left)
                    height = max(0, bottom - top)

                    frame = protocol.get_color_data(
                        left,
                        top,
                        width,
                        height,
                    )
                    uart.send(frame)
                    last_send_ms = now_ms
                    uart_send_count += 1

                    # 避免每帧打印，约每 25 帧输出一次实际发送内容。
                    if (uart_send_count % 25) == 0:
                        print("UART TX:", frame)

            if DEBUG_SHOW_MASK:
                show_preview(mask, image.GRAYSCALE)
            else:
                # 先在 320x240 原图上绘制结果，再统一按配置显示。
                draw_result(
                    img,
                    found,
                    cx,
                    cy,
                    radius,
                    area,
                    circularity,
                    fps,
                    tracking_enabled,
                )
                show_preview(img.to_numpy_ref(), image.RGB888)

            frame_count += 1
            if frame_count >= 10:
                gc.collect()
                frame_count = 0

    except KeyboardInterrupt as exc:
        print("user stop:", exc)
    except BaseException as exc:
        # IDE 手动停止时会抛出 IDE interrupt，这属于正常停止，不打印。
        if "IDE interrupt" not in str(exc):
            print("exception:", exc)
    finally:
        if uart is not None:
            uart.deinit()
        if isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)


if __name__ == "__main__":
    main()
