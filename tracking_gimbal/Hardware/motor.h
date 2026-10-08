/**
 * motor.h - 步进云台 PWM 后端接口
 *
 * 该文件提供速度/位置两种上层逻辑共用的底层接口：
 *   - Motor_SetAxisSpeed_RPM(): 按 RPM 设置某一轴 STEP 频率。
 *   - Motor_StopAxis(): 停止某一轴 STEP，并保持 EN 使能锁定。
 *   - Motor_EnableStepInterrupt(): 位置或速度模式开始输出时打开 PWM 周期计步中断。
 *   - Motor_DisableStepInterrupt(): 到位或停止时关闭计步中断。
 */
#ifndef _MOTOR_H
#define _MOTOR_H
#include "ti_msp_dl_config.h"
#include "board.h"

#define GIMBAL_AXIS_X           1U
#define GIMBAL_AXIS_Y           2U

/* X/Y 轴 STEP 均使用 SysConfig 中的 C1 通道，不使用 C0/PA15/PA0。 */
#define GIMBAL_X_STEP_TIMER      PWM_0_INST
#define GIMBAL_X_STEP_CC_INDEX   DL_TIMER_CC_1_INDEX
#define GIMBAL_X_STEP_CC_OUTPUT  DL_TIMER_CC1_OUTPUT
#define GIMBAL_Y_STEP_TIMER      PWM_1_INST
#define GIMBAL_Y_STEP_CC_INDEX   DL_TIMER_CC_1_INDEX
#define GIMBAL_Y_STEP_CC_OUTPUT  DL_TIMER_CC1_OUTPUT

/*
 * 速度换算参数：
 *   3200 STEP/圈 = 200整步/圈 * 16细分。
 *   PWM 定时器时钟由 SysConfig 设置为 1MHz。
 *   PWM周期计数 = 1MHz * 60 / (RPM * 3200)。
 */
#define MOTOR_STEPS_PER_REV  3200U
#define PWM_TIMER_CLK_HZ     1000000U
#define SPEED_CONST_FACTOR   60000000.0f
#define MIN_PWM_PERIOD       100U
#define MAX_PWM_PERIOD       65535U
#define STOP_PWM_PERIOD      1000U       /* 停止时保留 1kHz 空闲周期，输出仍被强制为低。 */
#define SPEED_LIMIT          160.0f

extern volatile uint16_t Step_Period_X;
extern volatile uint16_t Step_Period_Y;

extern volatile float Target_Speed_X;
extern volatile float Target_Speed_Y;

uint16_t Motor_RpmToPeriodTicks(float abs_rpm);
void Motor_SetAxisSpeed_RPM(uint8_t axis, float rpm);
void Motor_StopAxis(uint8_t axis);
void Motor_EnableStepInterrupt(uint8_t axis);
void Motor_DisableStepInterrupt(uint8_t axis);
void Set_Motor_Speed_RPM(float rpm_x, float rpm_y);

#endif
