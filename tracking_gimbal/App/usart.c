/*
 * usart.c - 双 UART 通信实现
 *
 * 本文件属于“硬件通信层”，只处理串口字节，不负责解释协议含义。
 *
 * UART0 接收方向：
 *   K230 -> PA31 -> UART0 RX 中断
 *   -> UART_0_INST_IRQHandler()
 *   -> 读取字节并交给 Pto_Data_Receive()
 *
 * UART1 调试方向：
 *   上层调用 Debug_Uart_SendChar() 或 Debug_Uart_SendString()
 *   -> 等待 UART1 硬件不忙
 *   -> 把字节写入 UART1 发送寄存器
 *   -> PA26 -> USB 转串口 -> 电脑
 *
 * 为什么不在这里解析 x/y/w/h：
 *   中断应当尽快返回。如果直接在中断中执行字符串转换、printf 或云台控制，
 *   会长时间占用 CPU，并影响电机 PWM 等其他中断的实时性。
 */
#include "usart.h"

#include "yb_protocol.h"

#define DEBUG_UART_INST    UART_1_INST
#define DEBUG_COMMAND_MAX  32U

volatile uint32_t Uart0_RxByteCount = 0U;

static char DebugCommandBuffer[DEBUG_COMMAND_MAX];
static uint8_t DebugCommandLength = 0U;
static volatile uint8_t DebugCommandReady = 0U;

static void Debug_Uart_SendUnsigned(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    if (value == 0U) {
        Debug_Uart_SendChar((uint8_t)'0');
        return;
    }

    while ((value > 0U) && (count < sizeof(digits))) {
        digits[count] = (char)('0' + (value % 10U));
        value /= 10U;
        count++;
    }

    while (count > 0U) {
        count--;
        Debug_Uart_SendChar((uint8_t)digits[count]);
    }
}

/*
 * 初始化 UART0 的应用层中断入口。
 *
 * empty.syscfg 已经完成以下硬件配置：
 *   - 硬件外设选择 UART0
 *   - TX = PA28
 *   - RX = PA31
 *   - 波特率 = 115200
 *   - UART 内部 RX 中断使能
 *
 * 这里还需要打开 NVIC，让 UART0 中断能够真正进入 CPU。
 */
void Uart0_Init(void)
{
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

/*
 * 通过 UART1 发送一个调试字节。
 *
 * DL_UART_isBusy() 用来判断发送硬件是否还在处理上一条数据。
 * 忙时循环等待，空闲后才调用 transmitData() 写入新字节。
 */
void Debug_Uart_SendChar(uint8_t data)
{
    while (DL_UART_isBusy(DEBUG_UART_INST) == true) {
    }

    DL_UART_Main_transmitData(DEBUG_UART_INST, data);
}

/*
 * 通过 UART1 逐字符发送调试字符串。
 *
 * C 语言字符串以 '\0' 表示结束，所以循环条件同时检查指针和当前字符。
 * 例如 "ABC" 在内存中的字符序列是 'A'、'B'、'C'、'\0'。
 */
void Debug_Uart_SendString(const char *str)
{
    while ((str != 0) && (*str != '\0')) {
        Debug_Uart_SendChar((uint8_t)*str);
        str++;
    }
}

void Debug_Uart_Init(void)
{
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

/*
 * 从 UART1 中断缓冲区取出一条完整命令。
 *
 * 复制期间短暂关闭 UART1 中断，保证主循环不会读到正在变化的字符串。
 * 取走后清空长度和 ready 标志，下一次 UART1 中断可以继续收集新命令。
 */
uint8_t Debug_Uart_GetCommandLine(char *line, uint8_t max_len)
{
    uint8_t index;
    uint8_t available = 0U;

    if ((line == 0) || (max_len < 2U)) {
        return 0U;
    }

    NVIC_DisableIRQ(UART_1_INST_INT_IRQN);
    if (DebugCommandReady != 0U) {
        for (index = 0U;
             (index < DebugCommandLength) && (index < (max_len - 1U));
             index++) {
            line[index] = DebugCommandBuffer[index];
        }
        line[index] = '\0';

        DebugCommandLength = 0U;
        DebugCommandReady = 0U;
        available = 1U;
    }
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    return available;
}

/* 查询 K230 到 MSPM0 的物理接收字节数和有效目标帧数。 */
void Debug_Uart_ReportUart0Rx(void)
{
    Debug_Uart_SendString("UART0 RX bytes=");
    Debug_Uart_SendUnsigned(Uart0_RxByteCount);
    Debug_Uart_SendString(" frames=");
    Debug_Uart_SendUnsigned(Pto_GetObservationCount());
    Debug_Uart_SendString("\r\n");
}

/*
 * UART0 中断服务函数。
 *
 * UART_0_INST_IRQHandler 是 SysConfig 生成的宏，本工程中展开为
 * UART0_IRQHandler。函数名不能随意修改，否则启动文件中的中断向量
 * 无法找到这个函数。
 *
 * DL_UART_getPendingInterrupt() 会返回当前中断类型，并清除对应标志。
 * 当前应用只关心 RX，也就是“收到一个字节”的事件。
 */
void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_IIDX_RX:
            /*
             * receiveData() 把 UART 接收寄存器中的字节取出。
             * 这里不做字符串判断，立即交给协议层状态机收集。
             */
            Uart0_RxByteCount++;
            Pto_Data_Receive((uint8_t)DL_UART_Main_receiveData(UART_0_INST));
            break;

        default:
            break;
    }
}

/*
 * UART1 中断接收电脑发来的调试命令。
 *
 * 中断只负责逐字符存入缓冲区，并在收到换行时置 ready。
 * 字符串比较、TRACK/STOP 等操作留给主循环，避免阻塞中断。
 */
void UART_1_INST_IRQHandler(void)
{
    char rx_data;

    switch (DL_UART_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_IIDX_RX:
            rx_data = (char)DL_UART_Main_receiveData(UART_1_INST);

            if (rx_data == '\r') {
                break;
            }

            if (rx_data == '\n') {
                if (DebugCommandLength > 0U) {
                    DebugCommandBuffer[DebugCommandLength] = '\0';
                    DebugCommandReady = 1U;
                }
                break;
            }

            if ((DebugCommandReady != 0U) ||
                (DebugCommandLength >= (DEBUG_COMMAND_MAX - 1U))) {
                break;
            }

            if ((rx_data >= 'a') && (rx_data <= 'z')) {
                rx_data = (char)(rx_data - 'a' + 'A');
            }
            DebugCommandBuffer[DebugCommandLength] = rx_data;
            DebugCommandLength++;
            break;

        default:
            break;
    }
}
