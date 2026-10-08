/*
 * yb_protocol.c - YbProtocol ASCII 协议接收、解析与调试输出
 *
 * 本文件属于“协议层”，负责解释串口字节流中的完整数据帧。
 *
 * 完整数据流：
 *
 *   UART RX 中断
 *     -> Pto_Data_Receive()
 *     -> 搜索帧头 '$'
 *     -> 把后续字符收集到 RxBuffer
 *     -> 遇到帧尾 '#'
 *     -> 复制到 CommandBuffer 并设置 New_CMD_flag
 *
 *   主循环 Pto_Loop()
 *     -> 检查 New_CMD_flag
 *     -> Pto_Data_Parse()
 *     -> 检查头尾、字段数、长度、功能 ID 和数值范围
 *     -> 保存到 LatestObservation
 *     -> 发送解析结果
 *
 * 这样分工的原因：
 *   - 中断只收集字节，执行时间短，不阻塞电机控制中断。
 *   - 主循环执行字符串解析和输出，允许使用较慢的操作。
 */
#include "yb_protocol.h"

#include <stdlib.h>
#include <string.h>
#include "usart.h"

/* 当前正在接收的原始字节。只由 UART 中断访问。 */
static uint8_t RxBuffer[PTO_BUF_LEN_MAX];

/* 已经接收完成、等待主循环解析的一整帧。 */
static uint8_t CommandBuffer[PTO_BUF_LEN_MAX];

/* RxBuffer 当前写入位置。 */
static uint8_t RxIndex;

/* 0 = 正在等待帧头，1 = 已经找到帧头，正在接收帧内容。 */
static uint8_t RxFlag;

/* CommandBuffer 中实际保存的字符数。 */
static uint8_t New_CMD_length;

/* 最近一次解析成功的目标位置。 */
static TargetObservation_t LatestObservation;

/* 成功解析的目标帧累计数量。 */
static volatile uint32_t ObservationCount = 0U;

/*
 * 最近一次从 K230 收到的跟踪控制命令。
 *
 * 0 = 没有命令，1 = START，2 = STOP。
 * 主循环通过 Pto_TakeControlCommand() 取走后立即清零。
 */
static volatile uint8_t PendingControlCommand = 0U;

/* 中断置 1，主循环处理完成后清 0。 */
volatile uint8_t New_CMD_flag;

/*
 * 发送无符号整数。
 *
 * TI 当前工程中的 printf 数字格式转换出现了只输出前缀、不输出数字的问题。
 * 因此这里不依赖 "%u"，而是主动把整数拆成十进制字符。
 *
 * 例如 value = 205：
 *   205 % 10 = 5，保存 '5'
 *    20 % 10 = 0，保存 '0'
 *     2 % 10 = 2，保存 '2'
 * 最后倒序发送，串口收到 "205"。
 */
static void Pto_SendUnsigned(uint32_t value)
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
 * 发送有符号整数。
 * 负数先发送 '-'，再把绝对值交给无符号整数输出函数。
 */
static void Pto_SendSigned(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        Debug_Uart_SendChar((uint8_t)'-');
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }

    Pto_SendUnsigned(magnitude);
}

/* 返回最近一次成功解析的目标位置。 */
const TargetObservation_t *Pto_GetObservation(void)
{
    return &LatestObservation;
}

uint32_t Pto_GetObservationCount(void)
{
    return ObservationCount;
}

/*
 * 取出待处理控制命令，并清除内部标志。
 * 返回 0 表示没有命令，返回 1/2 表示 START/STOP。
 */
uint8_t Pto_TakeControlCommand(void)
{
    uint8_t command = PendingControlCommand;

    PendingControlCommand = PTO_CONTROL_NONE;
    return command;
}

/*
 * 接收状态机。
 *
 * 参数 rx_data 是 UART 中断刚取出的一个字节。
 * 本函数只负责找到一帧的开始和结束，不执行 atoi 或打印。
 */
void Pto_Data_Receive(uint8_t rx_data)
{
    /*
     * 上一帧还没有被主循环取走时，先忽略新字节。
     * 这会主动丢弃期间的新帧，但不会覆盖正在处理的完整数据。
     */
    if (New_CMD_flag != 0U) {
        return;
    }

    /*
     * 状态 0：还没有找到帧头。
     * 串口噪声和上一帧结尾的换行符都会在这里被忽略。
     */
    if (RxFlag == 0U) {
        if (rx_data == PTO_HEAD) {
            RxBuffer[0] = PTO_HEAD;
            RxIndex = 1U;
            RxFlag = 1U;
        }
        return;
    }

    /*
     * 状态 1：已经找到 '$'，正在收集后续字符。
     * 如果一直没有 '#', 超过缓冲区上限就丢弃整次接收，等待下一个 '$'。
     */
    if (RxIndex >= PTO_BUF_LEN_MAX) {
        RxIndex = 0U;
        RxFlag = 0U;
        return;
    }

    RxBuffer[RxIndex] = rx_data;
    RxIndex++;

    /*
     * 收到 '#' 表示一整帧结束。
     * 先复制到 CommandBuffer，再设置 New_CMD_flag 通知主循环。
     * 顺序不能反过来，否则主循环可能在复制完成前就开始解析。
     */
    if (rx_data == PTO_TAIL) {
        memcpy(CommandBuffer, RxBuffer, RxIndex);
        New_CMD_length = RxIndex;
        New_CMD_flag = 1U;
        RxIndex = 0U;
        RxFlag = 0U;
    }
}

/*
 * 解析完整的 YbProtocol ASCII 帧。
 * 长度字段采用方案 A：包含帧头 '$' 和帧尾 '#'。
 */
uint8_t Pto_Data_Parse(const uint8_t *data_buf, uint8_t num)
{
    char frame[PTO_BUF_LEN_MAX + 1U];
    char *cursor;
    int values[PTO_FIELD_COUNT] = {0};
    uint8_t field_count = 0U;
    uint8_t function_id;

    if ((data_buf == 0) || (num < 5U) || (num > PTO_BUF_LEN_MAX) ||
        (data_buf[0] != PTO_HEAD) ||
        (data_buf[num - 1U] != PTO_TAIL)) {
        Debug_Uart_SendString("YB frame error: bad head/tail\r\n");
        return PTO_PARSE_RESULT_NONE;
    }

    memcpy(frame, data_buf, num);
    frame[num] = '\0';
    frame[num - 1U] = '\0';

    cursor = &frame[1];
    while ((*cursor != '\0') && (field_count < PTO_FIELD_COUNT)) {
        char *separator = strchr(cursor, ',');

        if (separator != 0) {
            *separator = '\0';
        }

        values[field_count] = atoi(cursor);
        field_count++;

        if (separator == 0) {
            break;
        }
        cursor = separator + 1;
    }

    if (field_count < 2U) {
        Debug_Uart_SendString("YB frame error: missing fields\r\n");
        return PTO_PARSE_RESULT_NONE;
    }

    if (values[0] != (int)num) {
        Debug_Uart_SendString("YB length error: field=");
        Pto_SendSigned((int32_t)values[0]);
        Debug_Uart_SendString(" actual=");
        Pto_SendUnsigned((uint32_t)num);
        Debug_Uart_SendString("\r\n");
        return PTO_PARSE_RESULT_NONE;
    }

    function_id = (uint8_t)values[1];

    /*
     * 控制帧不使用 x/y/w/h，只允许 3 个数字字段：
     * 长度、功能 ID、控制动作。
     */
    if (function_id == PTO_FUNC_ID_CONTROL) {
        if (field_count != 3U) {
            Debug_Uart_SendString("YB control error: fields\r\n");
            return PTO_PARSE_RESULT_NONE;
        }

        if ((values[2] != (int)PTO_CONTROL_START) &&
            (values[2] != (int)PTO_CONTROL_STOP)) {
            Debug_Uart_SendString("YB control error: value\r\n");
            return PTO_PARSE_RESULT_NONE;
        }

        PendingControlCommand = (uint8_t)values[2];
        return PTO_PARSE_RESULT_CONTROL;
    }

    if (function_id != PTO_FUNC_ID_COLOR) {
        Debug_Uart_SendString("YB function error: id=");
        Pto_SendSigned((int32_t)function_id);
        Debug_Uart_SendString("\r\n");
        return PTO_PARSE_RESULT_NONE;
    }

    if (field_count != PTO_FIELD_COUNT) {
        Debug_Uart_SendString("YB frame error: field count=");
        Pto_SendUnsigned((uint32_t)field_count);
        Debug_Uart_SendString("\r\n");
        return PTO_PARSE_RESULT_NONE;
    }

    if ((values[2] < 0) || (values[3] < 0) ||
        (values[4] < 0) || (values[5] < 0) ||
        (values[2] > 65535) || (values[3] > 65535) ||
        (values[4] > 65535) || (values[5] > 65535)) {
        Debug_Uart_SendString("YB value error: x=");
        Pto_SendSigned((int32_t)values[2]);
        Debug_Uart_SendString(" y=");
        Pto_SendSigned((int32_t)values[3]);
        Debug_Uart_SendString(" w=");
        Pto_SendSigned((int32_t)values[4]);
        Debug_Uart_SendString(" h=");
        Pto_SendSigned((int32_t)values[5]);
        Debug_Uart_SendString("\r\n");
        return PTO_PARSE_RESULT_NONE;
    }

    LatestObservation.x = (uint16_t)values[2];
    LatestObservation.y = (uint16_t)values[3];
    LatestObservation.w = (uint16_t)values[4];
    LatestObservation.h = (uint16_t)values[5];
    return PTO_PARSE_RESULT_OBSERVATION;
}

/*
 * 在主循环中反复调用。
 * 只有 UART 中断成功收齐一整帧时，New_CMD_flag 才会变成 1。
 */
void Pto_Loop(void)
{
    const TargetObservation_t *observation;
    uint32_t center_x;
    uint32_t center_y;
    uint8_t parse_result;

    if (New_CMD_flag == 0U) {
        return;
    }

    /*
     * 解析成功时打印四个目标参数。
     * 这里使用自定义数字发送函数，避免 printf 的 "%u" 在当前工具链中
     * 只输出文字前缀而不输出数字。
     */
    parse_result = Pto_Data_Parse(CommandBuffer, New_CMD_length);

    /*
     * 目标帧：更新观测计数，并打印调试信息。
     */
    if (parse_result == PTO_PARSE_RESULT_OBSERVATION) {
        ObservationCount++;
        observation = Pto_GetObservation();
        center_x = (uint32_t)observation->x +
            ((uint32_t)observation->w / 2U);
        center_y = (uint32_t)observation->y +
            ((uint32_t)observation->h / 2U);
        Debug_Uart_SendString("YB target: x=");
        Pto_SendUnsigned((uint32_t)observation->x);
        Debug_Uart_SendString(" y=");
        Pto_SendUnsigned((uint32_t)observation->y);
        Debug_Uart_SendString(" w=");
        Pto_SendUnsigned((uint32_t)observation->w);
        Debug_Uart_SendString(" h=");
        Pto_SendUnsigned((uint32_t)observation->h);
        Debug_Uart_SendString(" cx=");
        Pto_SendUnsigned(center_x);
        Debug_Uart_SendString(" cy=");
        Pto_SendUnsigned(center_y);
        Debug_Uart_SendString("\r\n");
    /*
     * 控制帧：这里只打印结果，真正的 Tracking_Start/Stop 由 empty.c
     * 通过 Pto_TakeControlCommand() 执行。
     */
    } else if (parse_result == PTO_PARSE_RESULT_CONTROL) {
        Debug_Uart_SendString("YB control: ");
        if (PendingControlCommand == PTO_CONTROL_START) {
            Debug_Uart_SendString("START\r\n");
        } else {
            Debug_Uart_SendString("STOP\r\n");
        }
    }

    /*
     * 无论本次解析成功还是失败，都释放接收门闩，
     * 允许 UART 中断开始接收下一帧。
     */
    New_CMD_flag = 0U;
    New_CMD_length = 0U;
}
