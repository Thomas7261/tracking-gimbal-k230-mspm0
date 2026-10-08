/*
 * usart.h - 串口应用层接口
 *
 * 两路 UART 分工：
 *   UART0 / PA31 RX：接收 K230 发来的 YbProtocol 数据帧。
 *   UART1 / PA26 TX：向电脑发送调试信息。
 *
 * UART0 使用 RX 中断收集字节；UART1 同时具备 TX 调试输出和 RX 命令输入。
 * 波特率、引脚和数据格式都由 empty.syscfg 配置，本文件不重复初始化。
 */
#ifndef _USART_H_
#define _USART_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* UART0 从 K230 收到的原始字节总数，用于阶段 3 联调诊断。 */
extern volatile uint32_t Uart0_RxByteCount;

/* 初始化 UART1 的调试接收中断。 */
void Debug_Uart_Init(void);

/*
 * 清除 UART0 的旧中断标志，并打开 NVIC 中 UART0 对应的中断通道。
 * 调用前必须已经执行 SYSCFG_DL_init()，因为 UART0 硬件外设和 RX
 * 中断使能位由 SysConfig 生成的代码负责初始化。
 */
void Uart0_Init(void);

/*
 * 通过 UART1 阻塞发送 1 个调试字节。
 * 如果 UART 正在发送上一条数据，函数会等待硬件空闲后再写入。
 */
void Debug_Uart_SendChar(uint8_t data);

/*
 * 通过 UART1 阻塞发送字符串，直到遇到字符串结束符 '\0'。
 */
void Debug_Uart_SendString(const char *str);

/*
 * 通过 UART1 输出当前 UART0 接收字节数。
 */
void Debug_Uart_ReportUart0Rx(void);

/*
 * 取出一条以换行结束的 UART1 调试命令。
 * 返回 1 表示 line 中已经写入一条完整命令，返回 0 表示当前没有新命令。
 */
uint8_t Debug_Uart_GetCommandLine(char *line, uint8_t max_len);

#endif
