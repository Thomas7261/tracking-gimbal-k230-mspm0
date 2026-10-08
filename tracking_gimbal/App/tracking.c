/*
 * tracking.c - 橙色球视觉跟踪控制实现
 *
 * 坐标约定：
 *   K230 图像 cx 向右为正，cy 向下为正。
 *   摄像头逆时针旋转 90 度后，A 轴使用 cy 误差，B 轴使用 cx 误差。
 *   实机确认的电机命令方向：
 *     A+：底座向右，A-：底座向左。
 *     B+：摄像头向上，B-：摄像头向下。
 *
 * 当前跟踪策略：
 *   K230 新帧 -> 计算 A/B 误差 -> 误差换算为目标 RPM
 *   -> 每 40ms 对 RPM 做一次斜率限制 -> 连续输出速度。
 *
 * 安全保护：
 *   - 正负 90 度软限位，按实际发出的 STEP 脉冲计数。
 *   - 目标丢失超过 200ms 时立即停止两轴。
 *   - 单轴 RPM 和变化速度均有限幅。
 */
#include "tracking.h"

#include <string.h>

#include "control.h"
#include "motor.h"
#include "usart.h"
#include "yb_protocol.h"

/* ============================== 可调参数 ==============================
 * 调参顺序建议：
 *   1. 先调 SPEED_GAIN，决定“同样误差下愿意转多快”；
 *   2. 再调 MAX_TRACKING_RPM，限制最高转速；
 *   3. 最后调 ERROR_RATE_LOOKAHEAD 和 SPEED_BRAKE，处理急停过冲。
 * 每次只改一个参数，并保留一次实机现象或串口记录。
 * ==================================================================== */
#define FRAME_CENTER_X                    160.0f
#define FRAME_CENTER_Y                    120.0f

#define DEADZONE_A                          8.0f
#define DEADZONE_B                          8.0f

/* 比例增益：每 1 像素误差对应多少 RPM。 */
#define SPEED_GAIN_A                        0.35f
#define SPEED_GAIN_B                        0.35f

/* 当前硬件的保守速度上限，后续可以按实机表现调整。 */
#define MAX_TRACKING_RPM_A                  30.0f
#define MAX_TRACKING_RPM_B                  30.0f
#define MIN_TRACKING_RPM                     3.0f

/* 加速可以慢一些，减速要更快，才能压住急停时的过冲。 */
#define SPEED_ACCEL_RPM_PER_CYCLE            3.0f
#define SPEED_BRAKE_RPM_PER_CYCLE            8.0f

/* 用误差变化率做约 80ms 的前瞻，抵消视觉和电机延迟。 */
#define ERROR_RATE_LOOKAHEAD_S               0.08f
#define ERROR_RATE_FILTER_ALPHA              0.35f
#define ERROR_RATE_LIMIT_PX_PER_S          600.0f

#define SOFT_LIMIT_DEG                      90.0f
#define SOFT_LIMIT_STEPS \
    ((int32_t)(SOFT_LIMIT_DEG * (float)MOTOR_STEPS_PER_REV / 360.0f))

#define CONTROL_PERIOD_MS                   40U
#define TARGET_LOST_TIMEOUT_MS             200U
#define TRACKING_MOVE_SPEED_RPM            35.0f
#define MANUAL_TEST_STEP_DEG               10.0f
#define MIN_MANUAL_STEP                      0.01f

static volatile uint32_t s_now_ms = 0U;

static uint32_t s_last_seen_count = 0U;
static uint32_t s_last_control_ms = 0U;
static uint32_t s_last_observation_ms = 0U;

static uint8_t s_enabled = 0U;
static uint8_t s_has_target = 0U;
static uint8_t s_observation_pending = 0U;

static float s_requested_speed_a = 0.0f;
static float s_requested_speed_b = 0.0f;
static float s_applied_speed_a = 0.0f;
static float s_applied_speed_b = 0.0f;

static uint32_t s_last_error_ms = 0U;
static uint8_t s_error_history_valid = 0U;
static float s_last_error_a = 0.0f;
static float s_last_error_b = 0.0f;
static float s_filtered_error_rate_a = 0.0f;
static float s_filtered_error_rate_b = 0.0f;

/* 返回绝对值，不依赖 math.h，减少嵌入式工程依赖。 */
static float Tracking_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/* 把浮点数限制在 [minimum, maximum] 区间。 */
static float Tracking_ClampFloat(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static void Tracking_SendUnsigned(uint32_t value)
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

static void Tracking_SendSigned(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        Debug_Uart_SendChar((uint8_t)'-');
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }

    Tracking_SendUnsigned(magnitude);
}

/*
 * 根据像素误差计算目标速度。
 * 误差越大速度越大，进入死区立即返回 0。
 */
static float Tracking_CalcTargetSpeed(float error, float deadzone,
    float gain, float max_rpm)
{
    float abs_error = Tracking_AbsFloat(error);
    float speed;

    if (abs_error <= deadzone) {
        return 0.0f;
    }

    speed = gain * abs_error;
    if (speed < MIN_TRACKING_RPM) {
        speed = MIN_TRACKING_RPM;
    }
    if (speed > max_rpm) {
        speed = max_rpm;
    }

    return (error < 0.0f) ? -speed : speed;
}

/*
 * 速度不能让云台继续向外越过软限位。
 * 只阻止“继续越界”的方向，反向运动始终允许。
 */
static float Tracking_ApplySoftLimit(uint8_t axis, float rpm)
{
    int32_t position_steps = Control_GetPositionSteps(axis);

    if ((rpm > 0.0f) && (position_steps >= SOFT_LIMIT_STEPS)) {
        return 0.0f;
    }
    if ((rpm < 0.0f) && (position_steps <= -SOFT_LIMIT_STEPS)) {
        return 0.0f;
    }

    return rpm;
}

/*
 * RPM 斜率限制。
 *
 * 加速和减速使用不同步长：
 *   - 加速慢，减少启动冲击；
 *   - 减速快，尽早制动，减小急停过冲。
 */
static float Tracking_SlewSpeed(float current, float target)
{
    float effective_target = target;
    float difference;
    float max_change;
    uint8_t accelerating;

    /*
     * 当前速度和目标速度方向相反时，本轮先减速到 0。
     * 下一轮再正式开始反向加速，避免突然反转换向。
     */
    if (((current > 0.0f) && (target < 0.0f)) ||
        ((current < 0.0f) && (target > 0.0f))) {
        effective_target = 0.0f;
    }

    difference = effective_target - current;

    accelerating =
        ((current == 0.0f) ||
         ((current > 0.0f) && (effective_target > 0.0f)) ||
         ((current < 0.0f) && (effective_target < 0.0f))) &&
        (Tracking_AbsFloat(effective_target) > Tracking_AbsFloat(current));
    max_change = (accelerating != 0U) ?
        SPEED_ACCEL_RPM_PER_CYCLE : SPEED_BRAKE_RPM_PER_CYCLE;

    if (difference > max_change) {
        return current + max_change;
    }
    if (difference < -max_change) {
        return current - max_change;
    }

    return effective_target;
}

/*
 * 真正向电机层写入速度。
 * 小于 0.5 RPM 时直接停止，避免超低速脉冲不稳定。
 */
static void Tracking_SetAxisSpeed(uint8_t axis, float rpm)
{
    if (Tracking_AbsFloat(rpm) < 0.5f) {
        Control_StopMotion(axis);
        return;
    }

    /*
     * 先打开计步中断，再启动 PWM，避免第一枚 STEP 脉冲漏计。
     * 中断中按实际输出的 STEP 脉冲更新软件位置。
     */
    Motor_EnableStepInterrupt(axis);
    Motor_SetAxisSpeed_RPM(axis, rpm);
}

static void Tracking_StopAxes(void)
{
    Control_StopMotion(GIMBAL_AXIS_X);
    Control_StopMotion(GIMBAL_AXIS_Y);

    s_requested_speed_a = 0.0f;
    s_requested_speed_b = 0.0f;
    s_applied_speed_a = 0.0f;
    s_applied_speed_b = 0.0f;

    s_error_history_valid = 0U;
    s_filtered_error_rate_a = 0.0f;
    s_filtered_error_rate_b = 0.0f;
}

/*
 * 根据一帧新目标计算 A/B 轴目标速度。
 *
 * 误差本身是 P 项；误差变化率是 D 类制动项。
 * 两者组合成“带预判的比例速度控制”，不依赖编码器。
 */
static void Tracking_UpdateRequestedSpeeds(
    const TargetObservation_t *observation)
{
    float cx;
    float cy;
    float error_a;
    float error_b;
    float predicted_error_a;
    float predicted_error_b;
    float raw_rate_a;
    float raw_rate_b;
    float delta_s;
    uint32_t now_ms;

    if (observation == 0) {
        return;
    }

    cx = (float)observation->x + ((float)observation->w * 0.5f);
    cy = (float)observation->y + ((float)observation->h * 0.5f);

    /*
     * 实机标定结果：
     *   A+ 使 cy 减小，因此 A 轴速度与 cy 误差同号。
     *   B+ 使 cx 减小，因此 B 轴速度与 cx 误差同号。
     */
    error_a = cy - FRAME_CENTER_Y;
    error_b = cx - FRAME_CENTER_X;

    now_ms = s_now_ms;
    if (s_error_history_valid == 0U) {
        s_error_history_valid = 1U;
        s_filtered_error_rate_a = 0.0f;
        s_filtered_error_rate_b = 0.0f;
    } else {
        delta_s = (float)(now_ms - s_last_error_ms) / 1000.0f;
        if ((delta_s >= 0.005f) && (delta_s <= 0.20f)) {
            raw_rate_a = (error_a - s_last_error_a) / delta_s;
            raw_rate_b = (error_b - s_last_error_b) / delta_s;

            raw_rate_a = Tracking_ClampFloat(
                raw_rate_a, -ERROR_RATE_LIMIT_PX_PER_S,
                ERROR_RATE_LIMIT_PX_PER_S);
            raw_rate_b = Tracking_ClampFloat(
                raw_rate_b, -ERROR_RATE_LIMIT_PX_PER_S,
                ERROR_RATE_LIMIT_PX_PER_S);

            s_filtered_error_rate_a =
                (ERROR_RATE_FILTER_ALPHA * raw_rate_a) +
                ((1.0f - ERROR_RATE_FILTER_ALPHA) *
                 s_filtered_error_rate_a);
            s_filtered_error_rate_b =
                (ERROR_RATE_FILTER_ALPHA * raw_rate_b) +
                ((1.0f - ERROR_RATE_FILTER_ALPHA) *
                 s_filtered_error_rate_b);
        }
    }

    s_last_error_a = error_a;
    s_last_error_b = error_b;
    s_last_error_ms = now_ms;

    predicted_error_a = error_a +
        (ERROR_RATE_LOOKAHEAD_S * s_filtered_error_rate_a);
    predicted_error_b = error_b +
        (ERROR_RATE_LOOKAHEAD_S * s_filtered_error_rate_b);

    /*
     * 前瞻项只允许把速度提前降到 0，不让它单独驱动反向运动。
     * 这样可以主动制动，又避免视觉噪声造成来回抖动。
     */
    if ((error_a > 0.0f) && (predicted_error_a < 0.0f)) {
        predicted_error_a = 0.0f;
    } else if ((error_a < 0.0f) && (predicted_error_a > 0.0f)) {
        predicted_error_a = 0.0f;
    }

    if ((error_b > 0.0f) && (predicted_error_b < 0.0f)) {
        predicted_error_b = 0.0f;
    } else if ((error_b < 0.0f) && (predicted_error_b > 0.0f)) {
        predicted_error_b = 0.0f;
    }

    s_requested_speed_a = Tracking_CalcTargetSpeed(
        predicted_error_a, DEADZONE_A, SPEED_GAIN_A, MAX_TRACKING_RPM_A);
    s_requested_speed_b = Tracking_CalcTargetSpeed(
        predicted_error_b, DEADZONE_B, SPEED_GAIN_B, MAX_TRACKING_RPM_B);
}

/*
 * 每个控制周期执行一次：
 *   目标速度 -> 斜率限制 -> 软限位 -> 写入电机。
 */
static void Tracking_UpdateControl(void)
{
    float next_a = Tracking_SlewSpeed(s_applied_speed_a, s_requested_speed_a);
    float next_b = Tracking_SlewSpeed(s_applied_speed_b, s_requested_speed_b);

    next_a = Tracking_ApplySoftLimit(GIMBAL_AXIS_X, next_a);
    next_b = Tracking_ApplySoftLimit(GIMBAL_AXIS_Y, next_b);

    if (Tracking_AbsFloat(next_a - s_applied_speed_a) >= 0.01f) {
        s_applied_speed_a = next_a;
        Tracking_SetAxisSpeed(GIMBAL_AXIS_X, s_applied_speed_a);
    }

    if (Tracking_AbsFloat(next_b - s_applied_speed_b) >= 0.01f) {
        s_applied_speed_b = next_b;
        Tracking_SetAxisSpeed(GIMBAL_AXIS_Y, s_applied_speed_b);
    }
}

/*
 * 位置模式手动命令仍然使用相对角度移动。
 * 限位根据已发出的 STEP 脉冲换算出的角度判断。
 */
static float Tracking_ClampManualStep(uint8_t axis, float requested_step)
{
    float current_angle = Control_GetPositionAngle(axis);
    float next_angle = current_angle + requested_step;

    if (next_angle > SOFT_LIMIT_DEG) {
        next_angle = SOFT_LIMIT_DEG;
    }
    if (next_angle < -SOFT_LIMIT_DEG) {
        next_angle = -SOFT_LIMIT_DEG;
    }

    return next_angle - current_angle;
}

static void Tracking_MoveManual(uint8_t axis, float requested_step)
{
    float allowed_step;

    Tracking_Stop();
    allowed_step = Tracking_ClampManualStep(axis, requested_step);

    if (Tracking_AbsFloat(allowed_step) < MIN_MANUAL_STEP) {
        Debug_Uart_SendString("CMD LIMIT\r\n");
        return;
    }

    Motor_Move_Angle(axis, allowed_step, TRACKING_MOVE_SPEED_RPM);
}

static void Tracking_ReportPosition(void)
{
    Debug_Uart_SendString("POS A_steps=");
    Tracking_SendSigned(Control_GetPositionSteps(GIMBAL_AXIS_X));
    Debug_Uart_SendString(" B_steps=");
    Tracking_SendSigned(Control_GetPositionSteps(GIMBAL_AXIS_Y));
    Debug_Uart_SendString(" A_rpm_x10=");
    Tracking_SendSigned((int32_t)(s_applied_speed_a * 10.0f));
    Debug_Uart_SendString(" B_rpm_x10=");
    Tracking_SendSigned((int32_t)(s_applied_speed_b * 10.0f));
    Debug_Uart_SendString(" A_rate_x10=");
    Tracking_SendSigned((int32_t)(s_filtered_error_rate_a * 10.0f));
    Debug_Uart_SendString(" B_rate_x10=");
    Tracking_SendSigned((int32_t)(s_filtered_error_rate_b * 10.0f));
    Debug_Uart_SendString(" limit_steps=");
    Tracking_SendSigned(SOFT_LIMIT_STEPS);
    Debug_Uart_SendString("\r\n");
}

void Tracking_Init(void)
{
    s_now_ms = 0U;
    s_last_seen_count = Pto_GetObservationCount();
    s_last_control_ms = 0U;
    s_last_observation_ms = 0U;
    s_has_target = 0U;
    s_observation_pending = 0U;
    s_requested_speed_a = 0.0f;
    s_requested_speed_b = 0.0f;
    s_applied_speed_a = 0.0f;
    s_applied_speed_b = 0.0f;
    s_enabled = 0U;

    Debug_Uart_SendString(
        "TRACK READY: A+/A-/B+/B-/STOP/TRACK/ZERO/POS?\r\n");
}

void Tracking_Tick5ms(void)
{
    s_now_ms += 5U;
}

/*
 * 主循环中的跟踪任务。
 *
 * 每帧目标通过 ObservationCount 判断是否更新；
 * 目标超过 200ms 未更新时立即停机。
 */
void Tracking_Loop(void)
{
    uint32_t now_ms;
    uint32_t observation_count;

    if (s_enabled == 0U) {
        return;
    }

    now_ms = s_now_ms;
    observation_count = Pto_GetObservationCount();

    if (observation_count != s_last_seen_count) {
        s_last_seen_count = observation_count;
        s_last_observation_ms = now_ms;
        s_has_target = 1U;
        s_observation_pending = 1U;
    }

    if ((s_has_target != 0U) &&
        ((uint32_t)(now_ms - s_last_observation_ms) >
         TARGET_LOST_TIMEOUT_MS)) {
        Tracking_StopAxes();
        s_has_target = 0U;
        s_observation_pending = 0U;
        Debug_Uart_SendString("TRACK TARGET LOST\r\n");
    }

    if ((uint32_t)(now_ms - s_last_control_ms) < CONTROL_PERIOD_MS) {
        return;
    }

    s_last_control_ms = now_ms;

    if (s_has_target == 0U) {
        return;
    }

    if (s_observation_pending != 0U) {
        Tracking_UpdateRequestedSpeeds(Pto_GetObservation());
        s_observation_pending = 0U;
    }

    Tracking_UpdateControl();
}

/* 进入跟踪模式，但不会主动让电机转动，需等待第一帧目标。 */
void Tracking_Start(void)
{
    Tracking_StopAxes();

    s_enabled = 1U;
    s_has_target = 0U;
    s_observation_pending = 0U;
    s_last_seen_count = Pto_GetObservationCount();
    s_last_control_ms = s_now_ms;
    s_last_observation_ms = s_now_ms;

    Debug_Uart_SendString("TRACK START\r\n");
}

/* 退出跟踪模式并立即停止两轴。 */
void Tracking_Stop(void)
{
    uint8_t was_enabled = s_enabled;

    s_enabled = 0U;
    s_has_target = 0U;
    s_observation_pending = 0U;

    Tracking_StopAxes();

    if (was_enabled != 0U) {
        Debug_Uart_SendString("TRACK STOP\r\n");
    }
}

uint8_t Tracking_IsEnabled(void)
{
    return s_enabled;
}

/*
 * 统一调试命令入口。
 *
 * UART1 手工命令和 K230 按键控制最终都进入这里，
 * 因此两种入口共享同一套 STOP/TRACK 安全逻辑。
 */
void Tracking_ProcessCommand(const char *command)
{
    if (command == 0) {
        return;
    }

    if (strcmp(command, "A+") == 0) {
        Tracking_MoveManual(GIMBAL_AXIS_X, MANUAL_TEST_STEP_DEG);
        Debug_Uart_SendString("CMD A+\r\n");
    } else if (strcmp(command, "A-") == 0) {
        Tracking_MoveManual(GIMBAL_AXIS_X, -MANUAL_TEST_STEP_DEG);
        Debug_Uart_SendString("CMD A-\r\n");
    } else if (strcmp(command, "B+") == 0) {
        Tracking_MoveManual(GIMBAL_AXIS_Y, MANUAL_TEST_STEP_DEG);
        Debug_Uart_SendString("CMD B+\r\n");
    } else if (strcmp(command, "B-") == 0) {
        Tracking_MoveManual(GIMBAL_AXIS_Y, -MANUAL_TEST_STEP_DEG);
        Debug_Uart_SendString("CMD B-\r\n");
    } else if (strcmp(command, "STOP") == 0) {
        Tracking_Stop();
    } else if (strcmp(command, "TRACK") == 0) {
        Tracking_Start();
    } else if (strcmp(command, "ZERO") == 0) {
        Tracking_Stop();
        Control_ResetPosition(GIMBAL_AXIS_X);
        Control_ResetPosition(GIMBAL_AXIS_Y);
        Debug_Uart_SendString("CMD ZERO\r\n");
    } else if (strcmp(command, "RX?") == 0) {
        Debug_Uart_ReportUart0Rx();
    } else if (strcmp(command, "POS?") == 0) {
        Tracking_ReportPosition();
    } else {
        Debug_Uart_SendString("CMD UNKNOWN\r\n");
    }
}
