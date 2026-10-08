/*
 * tracking.h - 视觉跟踪控制接口
 *
 * 摄像头相对原方向逆时针旋转了 90 度，因此控制层先把图像轴映射到
 * A/B 云台轴，再根据目标中心与画面中心的误差计算目标速度。
 *
 * 当前自动跟踪使用：
 *   像素误差 -> 比例速度 -> 误差变化率制动 -> RPM 斜率限制。
 */
#ifndef _TRACKING_H_
#define _TRACKING_H_

#include <stdint.h>

/* 初始化跟踪状态，默认关闭自动跟踪。 */
void Tracking_Init(void);

/* 由 5ms 定时器中断调用，累计跟踪时间。 */
void Tracking_Tick5ms(void);

/* 由主循环反复调用，处理新目标、控制周期和丢失超时。 */
void Tracking_Loop(void);

/* 开启或停止自动跟踪。 */
void Tracking_Start(void);
void Tracking_Stop(void);

/* 处理 UART1 或 K230 控制帧转换后的一条命令。 */
void Tracking_ProcessCommand(const char *command);

/* 返回当前是否允许自动跟踪。 */
uint8_t Tracking_IsEnabled(void);

#endif
