/*
 * board.h -- 板级支持包 头文件
 *           Board Support Package Header
 *
 * 编码 / Encoding: GBK
 *
 * 功能说明 / Description:
 *   通用类型定义、宏常量、延迟函数声明
 *   Common type definitions, macros, delay function declarations
 */

#ifndef _BOARD_H_
#define _BOARD_H_
#include "stdio.h"
#include "string.h"
#include "ti_msp_dl_config.h"
#include "led.h"
#include "key.h"
#include "motor.h"

/* 绝对值宏 / Absolute value macro */
#define ABS(a)      (a>0 ? a:(-a))

/* ---- 类型定义 / Type Definitions ---- */

/* 有符号整数 / Signed integers */
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

/* 常量有符号 / Const signed (只读 / Read Only) */
typedef const int32_t sc32;
typedef const int16_t sc16;
typedef const int8_t sc8;

/* 易变有符号 / Volatile signed */
typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

/* 易变常量有符号 / Volatile const signed (只读 / Read Only) */
typedef __I int32_t vsc32;
typedef __I int16_t vsc16;
typedef __I int8_t vsc8;

/* 无符号整数 / Unsigned integers */
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

/* 常量无符号 / Const unsigned (只读 / Read Only) */
typedef const uint32_t uc32;
typedef const uint16_t uc16;
typedef const uint8_t uc8;

/* 易变无符号 / Volatile unsigned */
typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

/* 易变常量无符号 / Volatile const unsigned (只读 / Read Only) */
typedef __I uint32_t vuc32;
typedef __I uint16_t vuc16;
typedef __I uint8_t vuc8;

/* ---- 小车类型枚举 / Car Type Enumeration ---- */
/* 小车型号的枚举定义 */
typedef enum
{
    Mec_Car = 0,    /* 麦克纳姆轮 / Mecanum wheel */
    Omni_Car,       /* 全向轮 / Omni wheel */
    Akm_Car,        /* 阿克曼转向 / Ackermann steering */
    Diff_Car,       /* 差速驱动 / Differential drive */
    FourWheel_Car,  /* 四轮驱动 / Four-wheel drive */
    Tank_Car        /* 履带驱动 / Tank drive */
} CarMode;

/* ---- 全局变量 / Global Variables ---- */

extern int Flag_Stop;

/* ---- SysTick 配置 / SysTick Configuration ---- */

/* Systick 最大计数值, 24位 / Max SysTick count value, 24-bit */
#define SysTickMAX_COUNT 0xFFFFFF

/* Systick 计数频率 (80MHz) / SysTick count frequency */
#define SysTickFre 80000000

/* 将 systick 的计数值转换为具体的时间单位
 * Convert SysTick count to time units
 */
#define SysTick_MS(x)  ((SysTickFre/1000U)*(uint32_t)(x))
#define SysTick_US(x)  ((SysTickFre/1000000U)*(uint32_t)(x))

/* ---- 函数声明 / Function Declarations ---- */

uint32_t Systick_getTick(void);      /* 读取 SysTick 当前计数值 / Read SysTick current count */
void delay_ms(uint32_t ms);          /* 毫秒级阻塞延迟 / Millisecond blocking delay */
void delay_us(uint32_t us);          /* 微秒级阻塞延迟 / Microsecond blocking delay */
void delay_1us(unsigned long __us);  /* 1us 延迟别名 / 1us delay alias */
void delay_1ms(unsigned long ms);    /* 1ms 延迟别名 / 1ms delay alias */

#endif  /* #ifndef _BOARD_H_ */
