# k230_uart_link_test.py - K230 到 MSPM0 的最小串口链路测试
#
# 本脚本不初始化摄像头，也不依赖橙球识别。
# 它只通过官方 YbUart/YbProtocol 周期发送三组已知目标坐标，
# 用于确认 K230 -> UART0/PA31 -> MSPM0 -> UART1/PA26 -> 电脑 的链路。
#
# 预期电脑调试串口依次出现：
#   YB target: x=20 y=30 w=40 h=50
#   YB target: x=140 y=110 w=40 h=40
#   YB target: x=250 y=180 w=30 h=30
#
# 链路测试完成后停止本脚本，再运行 k230_tracking_ball_uart.py。

import time

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart


BAUDRATE = 115200
SEND_INTERVAL_MS = 500

TEST_TARGETS = [
    (20, 30, 40, 50),
    (140, 110, 40, 40),
    (250, 180, 30, 30),
]


def main():
    uart = None

    try:
        uart = YbUart(baudrate=BAUDRATE)
        protocol = YbProtocol()
        index = 0

        print("K230 UART link test started")

        while True:
            x, y, w, h = TEST_TARGETS[index]
            frame = protocol.get_color_data(x, y, w, h)
            uart.send(frame)
            print("UART TX:", frame)

            index += 1
            if index >= len(TEST_TARGETS):
                index = 0

            time.sleep_ms(SEND_INTERVAL_MS)

    except KeyboardInterrupt as exc:
        print("user stop:", exc)
    except BaseException as exc:
        # CanMV IDE 点击停止时会抛出 IDE interrupt，属于正常停止。
        if "IDE interrupt" not in str(exc):
            print("exception:", exc)
    finally:
        if uart is not None:
            uart.deinit()


if __name__ == "__main__":
    main()
