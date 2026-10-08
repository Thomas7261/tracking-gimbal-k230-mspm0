/**
 * control.c - 步进云台位置控制实现
 *
 * 控制方法保持简单开环：
 *   1. 把角度换算成需要输出的 STEP 数。
 *   2. 按 Move_Speed 设置 PWM 输出频率。
 *   3. PWM 每完成一个周期就在中断中累计一步。
 *   4. 步数达到目标后停止对应轴 STEP 输出。
 *   5. 每一枚真正发出的 STEP 都累加到统一软件位置计数器。
 *
 * 注意：本例程没有编码器反馈，Current_Angle_X/Y 是按“已成功发出的 STEP”累加得到的估计角度。
 */
#include "control.h"
#include "motor.h"
#include "key.h"

Motor_Pos_t MotorX_Pos = {0U, 0U, 0U, 0.0f};
Motor_Pos_t MotorY_Pos = {0U, 0U, 0U, 0.0f};

float Current_Angle_X = 0.0f;
float Current_Angle_Y = 0.0f;
float Target_Angle_X = 0.0f;
float Target_Angle_Y = 0.0f;

float Angle_Step = 90.0f;   /* 每次按键触发的相对移动角度。 */
float Move_Speed = 30.0f;   /* 位置移动时使用的固定速度，单位 RPM。 */

/*
 * 统一软件位置，单位是 STEP 脉冲。
 * 位置模式和速度模式都会更新它，但它仍然不是编码器反馈，只是“已发脉冲数”。
 */
static volatile int32_t Position_Steps_X = 0;
static volatile int32_t Position_Steps_Y = 0;

static Motor_Pos_t *Control_GetAxisState(uint8_t axis)
{
    if (axis == GIMBAL_AXIS_X) {
        return &MotorX_Pos;
    }
    if (axis == GIMBAL_AXIS_Y) {
        return &MotorY_Pos;
    }
    return 0;
}

static void Control_AddPositionStep(uint8_t axis, int8_t direction)
{
    const float degrees_per_step = 360.0f / (float)MOTOR_STEPS_PER_REV;

    if (direction == 0) {
        return;
    }

    /*
     * Position_Steps 保存“已发出脉冲数”的连续坐标。
     * Current_Angle 是它换算出的角度，二者在每一步同时更新。
     */
    if (axis == GIMBAL_AXIS_X) {
        Position_Steps_X += (int32_t)direction;
        Current_Angle_X += (float)direction * degrees_per_step;
    } else if (axis == GIMBAL_AXIS_Y) {
        Position_Steps_Y += (int32_t)direction;
        Current_Angle_Y += (float)direction * degrees_per_step;
    }
}

/*
 * 位置模式：相对当前位置移动指定角度。
 *
 * 该函数会把角度换算成 STEP 数，启动 PWM，并打开计步中断。
 * 速度模式使用 motor.c 的 Motor_SetAxisSpeed_RPM()，不调用本函数。
 */
void Motor_Move_Angle(uint8_t axis, float angle, float speed_rpm)
{
    Motor_Pos_t *pos;
    float abs_angle = (angle >= 0.0f) ? angle : -angle;
    float signed_speed;
    uint32_t steps;

    pos = Control_GetAxisState(axis);
    if ((pos == 0) || (abs_angle < 0.01f)) {
        return;
    }

    if (speed_rpm < 1.0f) {
        speed_rpm = 1.0f;
    }
    if (speed_rpm > SPEED_LIMIT) {
        speed_rpm = SPEED_LIMIT;
    }

    /*
     * 角度转 STEP 脉冲数：
     *   steps = angle / 360 * 3200
     * 新 PWM 方案中一个 PWM 周期就是一个完整 STEP 脉冲，
     * 所以这里不再像旧“比较翻转模式”那样乘以 2。
     */
    steps = (uint32_t)((abs_angle / 360.0f) * (float)MOTOR_STEPS_PER_REV + 0.5f);
    if (steps == 0U) {
        return;
    }

    /* 如果该轴正在运动，先停止旧任务，再装载新的位置任务。 */
    Motor_StopAxis(axis);

    pos->target_steps = steps;
    pos->current_steps = 0U;
    pos->move_angle = angle;
    pos->is_moving = 1U;

    if (axis == GIMBAL_AXIS_X) {
        Target_Angle_X = Current_Angle_X + angle;
    } else {
        Target_Angle_Y = Current_Angle_Y + angle;
    }

    /* angle 的符号决定方向；speed_rpm 本身只表示速度大小。 */
    signed_speed = (angle >= 0.0f) ? speed_rpm : -speed_rpm;
    Motor_SetAxisSpeed_RPM(axis, signed_speed);
    Motor_EnableStepInterrupt(axis);
}

void Motor_Move_Relative(uint8_t axis, float angle_step)
{
    Motor_Move_Angle(axis, angle_step, Move_Speed);
}

float Get_Current_Angle(uint8_t axis)
{
    if (axis == GIMBAL_AXIS_X) {
        return Current_Angle_X;
    }
    if (axis == GIMBAL_AXIS_Y) {
        return Current_Angle_Y;
    }
    return 0.0f;
}

/* 读取统一软件位置，单位是 STEP。 */
int32_t Control_GetPositionSteps(uint8_t axis)
{
    if (axis == GIMBAL_AXIS_X) {
        return Position_Steps_X;
    }
    if (axis == GIMBAL_AXIS_Y) {
        return Position_Steps_Y;
    }
    return 0;
}

/* 把统一软件位置从 STEP 换算成角度。 */
float Control_GetPositionAngle(uint8_t axis)
{
    return (float)Control_GetPositionSteps(axis) * 360.0f /
        (float)MOTOR_STEPS_PER_REV;
}

/*
 * 把当前物理位置定义为软件 0 点。
 * 该操作不会强制电机移动，只清零软件计数和位置任务。
 */
void Control_ResetPosition(uint8_t axis)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    if (axis == GIMBAL_AXIS_X) {
        Position_Steps_X = 0;
        Current_Angle_X = 0.0f;
        Target_Angle_X = 0.0f;
        MotorX_Pos.target_steps = 0U;
        MotorX_Pos.current_steps = 0U;
        MotorX_Pos.is_moving = 0U;
        MotorX_Pos.move_angle = 0.0f;
    } else if (axis == GIMBAL_AXIS_Y) {
        Position_Steps_Y = 0;
        Current_Angle_Y = 0.0f;
        Target_Angle_Y = 0.0f;
        MotorY_Pos.target_steps = 0U;
        MotorY_Pos.current_steps = 0U;
        MotorY_Pos.is_moving = 0U;
        MotorY_Pos.move_angle = 0.0f;
    }

    if (primask == 0U) {
        __enable_irq();
    }
}

/*
 * 立即停止该轴，并取消尚未完成的位置任务。
 * 软件位置保持当前值，不会被清零。
 */
void Control_StopMotion(uint8_t axis)
{
    Motor_StopAxis(axis);

    if (axis == GIMBAL_AXIS_X) {
        MotorX_Pos.is_moving = 0U;
        MotorX_Pos.target_steps = 0U;
        MotorX_Pos.current_steps = 0U;
        Target_Angle_X = Current_Angle_X;
    } else if (axis == GIMBAL_AXIS_Y) {
        MotorY_Pos.is_moving = 0U;
        MotorY_Pos.target_steps = 0U;
        MotorY_Pos.current_steps = 0U;
        Target_Angle_Y = Current_Angle_Y;
    }
}

void Set_Angle_Step(float angle)
{
    if (angle < 0.0f) {
        angle = -angle;
    }
    Angle_Step = angle;
}

void Set_Move_Speed(float speed)
{
    if (speed < 1.0f) {
        speed = 1.0f;
    }
    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    }
    Move_Speed = speed;
}

/*
 * PWM 周期中断：每调用一次，代表硬件已经发出一枚 STEP。
 *
 * 两种模式共用这个中断：
 *   - pos->is_moving = 1：位置模式，按 target_steps 计步；
 *   - pos->is_moving = 0：速度模式，按 Target_Speed 正负更新位置。
 */
void Gimbal_PositionStepISR(uint8_t axis)
{
    Motor_Pos_t *pos = Control_GetAxisState(axis);
    float target_speed;
    int8_t direction;

    if (pos == 0) {
        return;
    }

    if (pos->is_moving != 0U) {
        direction = (pos->move_angle >= 0.0f) ? 1 : -1;
        Control_AddPositionStep(axis, direction);
        pos->current_steps++;

        if (pos->current_steps >= pos->target_steps) {
            pos->is_moving = 0U;
            Motor_StopAxis(axis);
        }
        return;
    }

    /*
     * 速度模式：PWM 周期中断每来一次，代表实际输出了一枚 STEP。
     * 方向由当前目标速度决定，停止时 Motor_StopAxis() 会把速度清 0。
     */
    target_speed = (axis == GIMBAL_AXIS_X) ? Target_Speed_X : Target_Speed_Y;
    if (target_speed > 0.0f) {
        Control_AddPositionStep(axis, 1);
    } else if (target_speed < 0.0f) {
        Control_AddPositionStep(axis, -1);
    }
}

void Pos_Key(void)
{
    uint8_t event = click_N_Double(0U);

    if (event == KEY_EVENT_SINGLE) {
        Motor_Move_Relative(GIMBAL_AXIS_X,  Angle_Step);
        Motor_Move_Relative(GIMBAL_AXIS_Y,  Angle_Step);
    } else if (event == KEY_EVENT_DOUBLE) {
        Motor_Move_Relative(GIMBAL_AXIS_X, -Angle_Step);
        Motor_Move_Relative(GIMBAL_AXIS_Y, -Angle_Step);
    }
}
