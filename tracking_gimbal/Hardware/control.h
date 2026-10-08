/**
 * control.h - 步进云台位置控制接口
 *
 * 位置控制层负责：
 *   - 位置模式：走多少角度；
 *   - 速度模式：统计已经实际发出的 STEP 脉冲；
 *   - 给上层提供统一的软件位置，供软限位和调试查询使用。
 * STEP 频率、DIR、EN 和 PWM 中断开关由 motor.c 负责。
 */
#ifndef __CONTROL_H
#define __CONTROL_H
#include "board.h"

/*
 * 单轴位置控制状态：
 *   target_steps  : 本次移动需要输出的 STEP 脉冲总数。
 *   current_steps : PWM 周期中断中累计的已输出 STEP 数。
 *   is_moving     : 1 表示该轴正在位置移动，0 表示停止或到位。
 *   move_angle    : 本次移动的角度，其符号决定每一枚 STEP 的计数方向。
 */
typedef struct {
    uint32_t target_steps;
    uint32_t current_steps;
    uint8_t  is_moving;
    float    move_angle;
} Motor_Pos_t;

extern float Angle_Step;
extern float Move_Speed;

extern float Current_Angle_X;
extern float Current_Angle_Y;
extern float Target_Angle_X;
extern float Target_Angle_Y;

extern Motor_Pos_t MotorX_Pos;
extern Motor_Pos_t MotorY_Pos;

void Motor_Move_Angle(uint8_t axis, float angle, float speed_rpm);
void Motor_Move_Relative(uint8_t axis, float angle_step);
float Get_Current_Angle(uint8_t axis);

/* 统一软件位置接口：单位是 STEP，不是编码器真实角度。 */
int32_t Control_GetPositionSteps(uint8_t axis);
float Control_GetPositionAngle(uint8_t axis);
void Control_ResetPosition(uint8_t axis);
void Control_StopMotion(uint8_t axis);
void Set_Angle_Step(float angle);
void Set_Move_Speed(float speed);
void Gimbal_PositionStepISR(uint8_t axis);
void Pos_Key(void);

#endif
