/*
 * led.h -- LED 驱动模块 头文件
 *         LED Driver Module Header
 *
 * 编码 / Encoding: GBK
 */

#ifndef _LED_H
#define _LED_H
#include "ti_msp_dl_config.h"

/* LED 亮 (低电平有效) / LED ON (active low) */
void LED_ON(void);

/* LED 灭 / LED OFF */
void LED_OFF(void);

/* LED 翻转 / LED Toggle */
void LED_Toggle(void);

/* LED 闪烁 (基于调用频率自动翻转)
 * LED Flash (auto-toggles based on call frequency)
 * time: 翻转周期 (调用次数) / Toggle period (in call counts)
 */
void LED_Flash(uint16_t time);
#endif
