/*
 * led.c -- LED 驱动模块 实现
 *         LED Driver Module Implementation
 *
 * 编码 / Encoding: GBK
 *
 * 功能说明 / Description:
 *   LED 控制 (PB.9, 低电平有效)
 *   LED control (PB.9, active low)
 *   LED_Flash: 基于调用频率的自动闪烁
 *              Auto-flash based on call frequency
 */

#include "led.h"

/*
 * LED 亮 (低电平有效) / LED ON (active low)
 */
void LED_ON(void)
{
    DL_GPIO_clearPins(LED_PORT,LED_led_PIN);
}

/*
 * LED 灭 / LED OFF
 */
void LED_OFF(void)
{
    DL_GPIO_setPins(LED_PORT,LED_led_PIN);
}

/*
 * LED 翻转 / LED Toggle
 */
void LED_Toggle(void)
{
    DL_GPIO_togglePins(LED_PORT,LED_led_PIN);
}

/*
 * LED 自动闪烁 / LED Auto Flash
 * time: 翻转周期 (函数调用次数) / Toggle period in function call counts
 *       如 time=200, 调用频率200Hz, 则闪烁周期=1秒
 *       e.g. time=200, call rate=200Hz -> flash period=1 second
 */
void LED_Flash(uint16_t time)
{
    static uint16_t temp;
    if(time==0) LED_ON();
    else if(++temp==time) LED_Toggle(),temp=0;
}
