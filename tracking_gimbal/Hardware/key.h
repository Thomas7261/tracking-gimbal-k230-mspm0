/**
 * key.h - 位置控制按键接口
 *
 * 按键高电平有效。当前只识别单击和双击：
 *   empty.c 使用单击停止自动跟踪，使用双击重新开始跟踪。
 * 不再保留长按功能，避免长按与双击状态机互相干扰。
 */
#ifndef _KEY_H
#define _KEY_H
#include "ti_msp_dl_config.h"
#include "board.h"

#define KEY_PRESSED()  (DL_GPIO_readPins(KEY_PORT, KEY_key_PIN) > 0U)

#define KEY_EVENT_NONE        0U
#define KEY_EVENT_SINGLE      1U
#define KEY_EVENT_DOUBLE      2U

/* time 参数为旧接口兼容保留，当前内部使用固定双击窗口。 */
uint8_t click_N_Double(uint8_t time);
void Key(void);

#endif
