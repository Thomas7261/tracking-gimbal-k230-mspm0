/*
 * board.c -- 板级支持包 实现
 *           Board Support Package Implementation
 *
 * 编码 / Encoding: GBK
 *
 * 功能说明 / Description:
 *   SysTick 微秒级延迟、printf 串口重定向
 *   SysTick microsecond delay, printf UART redirect
 */

#include "ti_msp_dl_config.h"
#include "board.h"

volatile unsigned long tick_ms;
volatile uint32_t start_time;

/*
 * SysTick 初始化 (1ms 中断周期)
 * SysTick initialization (1ms interrupt period)
 */
void SysTick_Init(void)
{
    DL_SYSTICK_config(CPUCLK_FREQ/1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}

/*
 * 返回 SysTick 当前计数值 (递减计数器)
 * Return current SysTick count value (down-counter)
 */
uint32_t Systick_getTick(void)
{
    return (SysTick->VAL);
}

/*
 * 毫秒级阻塞延迟
 * Millisecond blocking delay
 */
void delay_ms(uint32_t ms)
{
    /* 超出能满足的最大延迟 / Exceeds maximum possible delay */
    /* if( ms > SysTickMAX_COUNT/(SysTickFre/1000) ) ms = SysTickMAX_COUNT/(SysTickFre/1000); */
    for(int i=0;i<1000;i++)
    {
        delay_us(ms);
    }
}

/*
 * 微秒级阻塞延迟 (基于 SysTick 递减计数器)
 * Microsecond blocking delay (based on SysTick down-counter)
 */
void delay_us(uint32_t us)
{
    /* 限制延迟上限 / Limit maximum delay */
    if( us > SysTickMAX_COUNT/(SysTickFre/1000000) ) us = SysTickMAX_COUNT/(SysTickFre/1000000);

    us = us*(SysTickFre/1000000); /* 单位转换 / Unit conversion */

    /* 用于保存已走过的时间 / Accumulated elapsed time */
    uint32_t runningtime = 0;

    /* 获得当前时刻的计数值 / Capture current count value */
    uint32_t InserTick = Systick_getTick();

    /* 用于刷新实时时间 / Used to refresh current time */
    uint32_t tick = 0;

    uint8_t countflag = 0;

    /* 等待延迟完成 / Wait for delay to complete */
    while(1)
    {
        tick = Systick_getTick();               /* 刷新当前时刻计数值 / Refresh current count */

        if( tick > InserTick ) countflag = 1;    /* 出现溢出轮询, 则切换走时的计算方式
                                                 * Overflow detected, switch time calculation mode */

        if( countflag ) runningtime = InserTick + SysTickMAX_COUNT - tick;
        else runningtime = InserTick - tick;

        if( runningtime>=us ) break;
    }

}

/* 1us/1ms 延迟别名函数 / 1us/1ms delay alias functions */
void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }

#if !defined(__MICROLIB)
/*
 * 不使用微库的话就需要添加下面的函数
 * The following functions are needed when not using MicroLib
 */
#if (__ARMCLIB_VERSION <= 6000000)
/* 如果编译器是 AC5 就定义下面这个结构体
 * Define this struct if compiler is AC5
 */
struct __FILE
{
    int handle;
};
#endif

FILE __stdout;

/*
 * 定义 _sys_exit() 以避免使用半主机模式
 * Define _sys_exit() to avoid semihosting mode
 */
void _sys_exit(int x)
{
    x = x;
}
#endif

/*
 * printf 重定向到 UART1（电脑调试口）
 * printf redirect to UART1 (PC debug port)
 *
 * 当串口忙的时候等待，不忙的时候再发送传进来的字符
 * Wait when UART is busy, transmit character when ready
 */
int fputc(int ch, FILE *stream)
{
    while( DL_UART_isBusy(UART_1_INST) == true );

    DL_UART_Main_transmitData(UART_1_INST, ch);

    return ch;
}
