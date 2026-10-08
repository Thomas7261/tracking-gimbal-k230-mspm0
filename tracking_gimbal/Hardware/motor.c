/**
 * motor.c - 步进云台 PWM 后端实现
 *
 * 本文件只负责“如何产生 STEP”：
 *   - 根据 RPM 计算 PWM 周期；
 *   - 按速度正负设置 DIR；
 *   - 启停 STEP 输出；
 *   - 按需打开/关闭 PWM 周期中断供位置层计步。
 *
 * 本文件不关心“要走多少角度”，角度到步数的换算放在 control.c。
 */
#include "motor.h"

volatile uint16_t Step_Period_X = 0U;
volatile uint16_t Step_Period_Y = 0U;

volatile float Target_Speed_X = 0.0f;
volatile float Target_Speed_Y = 0.0f;

/*
 * 把 RPM 换算成 PWM 自动重装载值。
 *
 * 例：30 RPM、3200 STEP/圈时：
 *   STEP/秒 = 30 * 3200 / 60 = 1600
 *   1MHz 定时器每 625 个计数输出一枚 STEP。
 */
uint16_t Motor_RpmToPeriodTicks(float abs_rpm)
{
    uint32_t period;

    if (abs_rpm < 0.5f) {
        return 0U;
    }

    period = (uint32_t)(SPEED_CONST_FACTOR /
        (abs_rpm * (float)MOTOR_STEPS_PER_REV));

    if (period < MIN_PWM_PERIOD) {
        period = MIN_PWM_PERIOD;
    }
    if (period > MAX_PWM_PERIOD) {
        period = MAX_PWM_PERIOD;
    }

    return (uint16_t)period;
}

/* 恢复 CCP 输出由 PWM OCTL 控制。停止后重新启动前必须恢复。 */
static void Motor_UsePWMOutput(GPTIMER_Regs *timer)
{
    DL_TimerA_setCCPOutputDisabled(timer,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL, DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);
}

/* 强制 STEP 为低电平，确保停止和上电锁定时不会误发脉冲。 */
static void Motor_ForceOutputLow(GPTIMER_Regs *timer)
{
    DL_TimerA_setCCPOutputDisabled(timer,
        DL_TIMER_CCP_DIS_OUT_LOW, DL_TIMER_CCP_DIS_OUT_LOW);
}

/* 写入某一路 PWM 的周期和 50% 占空比。 */
static void Motor_SetPWMTicks(GPTIMER_Regs *timer,
    DL_TIMER_CC_INDEX step_cc_index, uint16_t period_ticks)
{
    uint32_t load_value = (uint32_t)period_ticks - 1U;
    uint32_t duty_ticks = (uint32_t)period_ticks >> 1;

    DL_TimerA_stopCounter(timer);
    Motor_UsePWMOutput(timer);
    DL_TimerA_setLoadValue(timer, load_value);
    DL_TimerA_setTimerCount(timer, load_value);
    DL_TimerA_setCaptureCompareValue(timer, duty_ticks, step_cc_index);

    /* 另一路 CC 不作为 STEP 输出，放到周期末，降低配置残留影响。 */
    if (step_cc_index == DL_TIMER_CC_0_INDEX) {
        DL_TimerA_setCaptureCompareValue(timer, load_value, DL_TIMER_CC_1_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(timer, load_value, DL_TIMER_CC_0_INDEX);
    }
    DL_TimerA_startCounter(timer);
}

/* 停止 PWM 输出，但保持定时器可运行，方便下一次移动快速重新设置周期。 */
static void Motor_StopPWM(GPTIMER_Regs *timer, DL_TIMER_CC_INDEX step_cc_index)
{
    uint32_t load_value = STOP_PWM_PERIOD - 1U;

    DL_TimerA_stopCounter(timer);
    DL_TimerA_setLoadValue(timer, load_value);
    DL_TimerA_setTimerCount(timer, load_value);
    DL_TimerA_setCaptureCompareValue(timer, 0U, step_cc_index);
    Motor_ForceOutputLow(timer);
    DL_TimerA_startCounter(timer);
}

/* 打开某一轴的 PWM 周期中断，用于统计实际发出的 STEP 数。 */
void Motor_EnableStepInterrupt(uint8_t axis)
{
    if (axis == GIMBAL_AXIS_X) {
        DL_TimerA_clearInterruptStatus(PWM_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        DL_TimerA_enableInterrupt(PWM_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        NVIC_ClearPendingIRQ(PWM_0_INST_INT_IRQN);
        NVIC_EnableIRQ(PWM_0_INST_INT_IRQN);
    } else if (axis == GIMBAL_AXIS_Y) {
        DL_TimerA_clearInterruptStatus(PWM_1_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        DL_TimerA_enableInterrupt(PWM_1_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        NVIC_ClearPendingIRQ(PWM_1_INST_INT_IRQN);
        NVIC_EnableIRQ(PWM_1_INST_INT_IRQN);
    }
}

/* 关闭某一轴的 PWM 周期中断，通常与停止 STEP 一起使用。 */
void Motor_DisableStepInterrupt(uint8_t axis)
{
    if (axis == GIMBAL_AXIS_X) {
        DL_TimerA_disableInterrupt(PWM_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        DL_TimerA_clearInterruptStatus(PWM_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    } else if (axis == GIMBAL_AXIS_Y) {
        DL_TimerA_disableInterrupt(PWM_1_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        DL_TimerA_clearInterruptStatus(PWM_1_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    }
}

/*
 * 停止某一轴：
 *   - 关闭计步中断；
 *   - 把目标速度清 0；
 *   - 强制 STEP 输出低电平；
 *   - 保持 EN 有效，让云台停在当前位置。
 */
void Motor_StopAxis(uint8_t axis)
{
    if (axis == GIMBAL_AXIS_X) {
        Motor_DisableStepInterrupt(GIMBAL_AXIS_X);
        Step_Period_X = 0U;
        Target_Speed_X = 0.0f;
        Motor_StopPWM(GIMBAL_X_STEP_TIMER, GIMBAL_X_STEP_CC_INDEX);
        DL_GPIO_setPins(EN_PORT, EN_MA_PIN);
    } else if (axis == GIMBAL_AXIS_Y) {
        Motor_DisableStepInterrupt(GIMBAL_AXIS_Y);
        Step_Period_Y = 0U;
        Target_Speed_Y = 0.0f;
        Motor_StopPWM(GIMBAL_Y_STEP_TIMER, GIMBAL_Y_STEP_CC_INDEX);
        DL_GPIO_setPins(EN_PORT, EN_MB_PIN);
    }
}

/*
 * 连续速度控制入口。
 *
 * rpm > 0：正方向；rpm < 0：反方向；绝对值决定 STEP 频率。
 * 该函数只负责输出脉冲，不判断软限位，限位由 tracking.c 提前处理。
 */
void Motor_SetAxisSpeed_RPM(uint8_t axis, float rpm)
{
    float abs_speed;
    uint16_t new_period;

    if (rpm > SPEED_LIMIT) {
        rpm = SPEED_LIMIT;
    }
    if (rpm < -SPEED_LIMIT) {
        rpm = -SPEED_LIMIT;
    }

    abs_speed = (rpm >= 0.0f) ? rpm : -rpm;
    if (abs_speed < 0.5f) {
        Motor_StopAxis(axis);
        return;
    }

    new_period = Motor_RpmToPeriodTicks(abs_speed);

    if (axis == GIMBAL_AXIS_X) {
        if (rpm > 0.0f) {
            DL_GPIO_setPins(DIR_PORT, DIR_A_PIN);
        } else {
            DL_GPIO_clearPins(DIR_PORT, DIR_A_PIN);
        }
        DL_GPIO_setPins(EN_PORT, EN_MA_PIN);
        Step_Period_X = new_period;
        Target_Speed_X = rpm;
        Motor_SetPWMTicks(GIMBAL_X_STEP_TIMER, GIMBAL_X_STEP_CC_INDEX, new_period);
    } else if (axis == GIMBAL_AXIS_Y) {
        if (rpm > 0.0f) {
            DL_GPIO_setPins(DIR_PORT, DIR_B_PIN);
        } else {
            DL_GPIO_clearPins(DIR_PORT, DIR_B_PIN);
        }
        DL_GPIO_setPins(EN_PORT, EN_MB_PIN);
        Step_Period_Y = new_period;
        Target_Speed_Y = rpm;
        Motor_SetPWMTicks(GIMBAL_Y_STEP_TIMER, GIMBAL_Y_STEP_CC_INDEX, new_period);
    }
}

/* 兼容速度控制接口：位置模式内部主要使用 Motor_SetAxisSpeed_RPM。 */
void Set_Motor_Speed_RPM(float rpm_x, float rpm_y)
{
    Motor_SetAxisSpeed_RPM(GIMBAL_AXIS_X, rpm_x);
    Motor_SetAxisSpeed_RPM(GIMBAL_AXIS_Y, rpm_y);
}
