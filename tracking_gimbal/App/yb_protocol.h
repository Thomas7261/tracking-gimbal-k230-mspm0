/*
 * yb_protocol.h - YbProtocol ASCII 数据帧接口
 *
 * 帧格式：
 *
 *   $18,1,20,30,40,50#
 *   |  | |  |  |  |  |
 *   |  | |  |  |  |  +-- 帧尾 '#'
 *   |  | |  |  |  +----- h，目标高度
 *   |  | |  |  +-------- w，目标宽度
 *   |  | |  +----------- y，目标左上角 Y 坐标
 *   |  | +-------------- x，目标左上角 X 坐标
 *   |  +---------------- 功能 ID，1 表示颜色/目标位置
 *   +------------------- 长度，18 表示整帧共 18 个字符，包含 $ 和 #
 *
 * 控制帧示例：
 *
 *   $09,24,1#
 *   功能 ID 24 表示跟踪控制，最后一个字段 1=开始跟踪，2=停止跟踪。
 *
 * 本文件只声明协议层接口，不包含 UART 硬件初始化。
 */
#ifndef _YB_PROTOCOL_H_
#define _YB_PROTOCOL_H_

#include <stdint.h>

/* 一帧允许占用的最大字符数，防止异常数据无限写入缓冲区。 */
#define PTO_BUF_LEN_MAX       (50U)

/* 单帧最多包含 6 个字段：长度、功能 ID、x、y、w、h。 */
#define PTO_FIELD_COUNT       (6U)

/* ASCII 字符 '$' 和 '#' 的字节值。 */
#define PTO_HEAD              (0x24U)
#define PTO_TAIL              (0x23U)

/* 颜色/目标位置数据的功能 ID。 */
#define PTO_FUNC_ID_COLOR     (1U)

/* K230 发来的跟踪控制功能 ID。 */
#define PTO_FUNC_ID_CONTROL   (24U)
#define PTO_CONTROL_NONE      (0U)
#define PTO_CONTROL_START     (1U)
#define PTO_CONTROL_STOP      (2U)

/* Pto_Data_Parse() 的解析结果。 */
#define PTO_PARSE_RESULT_NONE         (0U)
#define PTO_PARSE_RESULT_OBSERVATION  (1U)
#define PTO_PARSE_RESULT_CONTROL      (2U)

/*
 * 解析成功后保存的最新目标位置。
 *
 * (x, y) 是目标外框左上角，(w, h) 是目标宽度和高度。
 * 坐标系来自 K230 图像，后续云台控制会使用这些数据计算目标中心。
 */
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
} TargetObservation_t;

/*
 * 完整帧就绪标志。
 *
 * 该变量会被 UART 中断和主循环共同访问，因此使用 volatile，
 * 防止编译器错误地把它缓存到寄存器中。
 */
extern volatile uint8_t New_CMD_flag;

/*
 * UART 中断每收到一个字节就调用一次。
 * 函数负责把字节收集成完整帧，不负责解析数字。
 */
void Pto_Data_Receive(uint8_t rx_data);

/*
 * 解析一条完整帧。
 * 返回 PTO_PARSE_RESULT_* 表示解析结果。
 */
uint8_t Pto_Data_Parse(const uint8_t *data_buf, uint8_t num);

/*
 * 在主循环中调用。
 * 当 New_CMD_flag 有效时，取出完整帧、解析并打印结果。
 */
void Pto_Loop(void);

/*
 * 返回最近一次解析成功的目标位置只读指针。
 */
const TargetObservation_t *Pto_GetObservation(void);

/*
 * 返回成功解析的目标帧累计数量。
 * 跟踪模块通过比较计数判断是否出现了新目标。
 */
uint32_t Pto_GetObservationCount(void);

/*
 * 取出 K230 发来的一条跟踪控制命令。
 * 返回 0 表示当前没有待处理命令。
 */
uint8_t Pto_TakeControlCommand(void);

#endif
