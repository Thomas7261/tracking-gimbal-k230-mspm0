# k230_tracking_ball_color.py - K230 彩色球识别实验
#
# 当前阶段只验证视觉识别：
#   摄像头 -> HSV 颜色分割 -> 形态学去噪 -> 轮廓筛选
#   -> 最小外接圆 -> IDE 显示并打印目标坐标
#
# 本脚本暂不发送 UART 数据。
#
# ============================================================================
# 调参入口说明（先看这里）
# ============================================================================
#
# 第 1 步：修改 TARGET_COLOR，选择与你实际球颜色一致的模式。
#
# 第 2 步：观察 IDE 中颜色区域能否完整覆盖球。
#           颜色区域太小：放宽对应的 HSV 范围。
#           背景也被选中：收紧对应的 HSV 范围。
#
# 第 3 步：调 MIN_CONTOUR_AREA。
#           小球漏检：减小；背景碎片误检：增大。
#
# 第 4 步：调 MIN_CIRCULARITY 和 MAX_ASPECT_RATIO。
#           圆形越接近球，circularity 越接近 1.0。
#           手遮挡球时轮廓可能变形，可以把下限稍微放宽。
#
# 第 5 步：调 MIN_RADIUS 和 MAX_RADIUS。
#           它们限制最终最小外接圆的半径。
#
# ============================================================================

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


# ============================================================================
# 【调参位置 1】目标颜色
# ============================================================================
#
# 修改 TARGET_COLOR，只填写字典中的一个键：
#   "red"、"orange"、"yellow"、"green"、"blue"
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
        # 固定阈值来自实测校准：
        # CAL HSV mean: H=7.2 S=216.3 V=198.6
        # CAL HSV range: H=0..17 S=171..255 V=153..255
        # 这里略微放宽，减少开机位置和轻微光照变化造成的漏检。
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

# ============================================================================
# 【调参位置 1.5】启动自动颜色校准
# ============================================================================
#
# 自动校准会要求你把球放在画面正中央。
# 程序在中央 40x40 区域连续采样若干帧，计算橙色球的平均 H、S、V，
# 然后自动生成 HSV 阈值。
#
# 当前已经保存了实测成功的固定阈值，因此默认关闭每次启动校准。
# 如果更换灯光、背景或相机曝光，再把它改回 True 重新校准一次。
#
# 注意：
#   - 校准时画面中央必须只有球，不要把白纸、桌面或手放进中央区域。
#   - 校准时尽量保持与实际跟踪时相同的灯光。
#   - 校准完成后，程序会自动切到颜色掩码调试画面。
AUTO_CALIBRATE = False
CALIBRATION_FRAMES = 30
CALIBRATION_ROI_HALF = 20

# 自动阈值允许偏离平均值的范围。
CAL_H_TOLERANCE = 10
CAL_S_MARGIN = 45
CAL_V_MARGIN = 45


# ============================================================================
# 【调参位置 2】图像尺寸和形态学
# ============================================================================
#
# 继续使用 320x240 RGB888：
#   - 与 MSPM0 的目标坐标空间一致。
#   - 与当前 OpenCV 霍夫圆脚本保持一致。
DETECT_WIDTH = 320
DETECT_HEIGHT = 240
FRAME_CENTER_X = DETECT_WIDTH // 2
FRAME_CENTER_Y = DETECT_HEIGHT // 2

SENSOR_WIDTH = 1280
SENSOR_HEIGHT = 960
SENSOR_FPS = 90

# 开运算先去掉孤立噪点，闭运算再填补颜色区域内部的小孔。
MORPH_KERNEL_SIZE = (5, 5)

# 调颜色阈值时建议保持 True：
#   True  -> IDE 显示黑白颜色掩码，白色像素表示“被 HSV 判定为橙色”。
#   False -> IDE 显示正常画面和检测圆。
#
# 如果白墙、白纸、白色反光在掩码中也显示为白色区域，说明阈值仍过宽。
# 如果橙球本身没有被选成白色区域，说明阈值过严。
DEBUG_SHOW_MASK = False


# ============================================================================
# 【调参位置 3】轮廓和圆形筛选
# ============================================================================
#
# r=15 的球面积约为：
#
#   area = pi * r^2 = 3.14 * 15 * 15 = 707 像素
#
# 手遮挡一部分后面积会减少，所以最小面积不能设置为 700。
# 当前值默认允许只看到约四分之一球的情况。
MIN_CONTOUR_AREA = 120

# 圆形度：
#
#   circularity = 4 * pi * area / perimeter^2
#
# 标准圆接近 1.0。
# 手遮挡后轮廓会变形，因此暂时允许较低的 0.45。
MIN_CIRCULARITY = 0.45

# 外接矩形的长边/短边比例。
# 球接近 1.0；超过该值说明轮廓明显细长，不像球。
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


def draw_result(img, found, cx, cy, radius, area, circularity, fps):
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


def main():
    sensor = None
    lost_frames = 0
    last_observation = None

    try:
        sensor = init_camera()
        Display.init(
            Display.ST7701,
            width=640,
            height=480,
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

            if DEBUG_SHOW_MASK:
                mask_img = image.Image(
                    DETECT_WIDTH,
                    DETECT_HEIGHT,
                    image.GRAYSCALE,
                    alloc=image.ALLOC_REF,
                    data=mask,
                )
                Display.show_image(mask_img)
            else:
                draw_result(
                    img,
                    found,
                    cx,
                    cy,
                    radius,
                    area,
                    circularity,
                    fps,
                )
                Display.show_image(img)

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
        if isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)


if __name__ == "__main__":
    main()
