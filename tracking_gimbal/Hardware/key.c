/**
 * key.c - 位置控制按键扫描
 *
 * 状态机说明：
 *   1. 按键 5ms 扫描一次，先用 20ms 消抖过滤机械抖动。
 *   2. 稳定释放一次算一次点击。
 *   3. 第一次点击后等待 200ms：
 *        - 如果窗口内出现第二次点击，则立即返回双击；
 *        - 如果窗口结束还没有第二次点击，则返回单击。
 *
 * 双击窗口越长，双击越容易；但单击确认也会相应变慢。
 */
#include "key.h"

#define KEY_DEBOUNCE_TICKS          4U      /* 4 * 5ms = 20ms */
#define KEY_DOUBLE_WINDOW_TICKS     40U     /* 40 * 5ms = 200ms */

static uint8_t Key_ScanEvent(void)
{
    static uint8_t stable_state = 0U;
    static uint8_t last_raw = 0U;
    static uint8_t debounce_count = 0U;
    static uint8_t click_count = 0U;
    static uint16_t release_ticks = 0U;

    uint8_t raw_state = KEY_PRESSED() ? 1U : 0U;
    uint8_t event = KEY_EVENT_NONE;

    if (raw_state == last_raw) {
        if (debounce_count < KEY_DEBOUNCE_TICKS) {
            debounce_count++;
        }
    } else {
        debounce_count = 0U;
        last_raw = raw_state;
    }

    if ((debounce_count >= KEY_DEBOUNCE_TICKS) && (stable_state != raw_state)) {
        stable_state = raw_state;
        if (stable_state == 0U) {
            click_count++;
            release_ticks = 0U;
            if (click_count >= 2U) {
                click_count = 0U;
                event = KEY_EVENT_DOUBLE;
            }
        }
    }

    if ((stable_state == 0U) && (click_count == 1U)) {
        release_ticks++;
        if (release_ticks >= KEY_DOUBLE_WINDOW_TICKS) {
            click_count = 0U;
            event = KEY_EVENT_SINGLE;
        }
    }

    return event;
}

u8 click_N_Double(u8 time)
{
    (void)time;
    return Key_ScanEvent();
}

void Key(void)
{
    (void)Key_ScanEvent();
}