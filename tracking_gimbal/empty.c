/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * empty.c - 步进云台跟踪主程序
 *
 * 硬件接线 / Wiring:
 *   X轴 STEP -> PWM_0 C1 / TIMA1_CCP1 / PA24
 *   Y轴 STEP -> PWM_1 C1 / TIMA0_CCP1 / PA22
 *   X轴 DIR  -> DIR_A / PA13
 *   Y轴 DIR  -> DIR_B / PA14
 *   X轴 EN   -> EN_MA / PA16
 *   Y轴 EN   -> EN_MB / PA17
 *
 * 当前软件分层：
 *   usart.c      : 收发字节，不做复杂解析；
 *   yb_protocol.c: 解析 K230 目标帧和控制帧；
 *   tracking.c   : 视觉误差 -> 目标 RPM -> 制动和限幅；
 *   control.c    : 位置模式、软件位置和 STEP 计数；
 *   motor.c      : PWM、DIR、EN 和 STEP 脉冲输出。
 *
 * 位置控制框架：
 *   1. control.c 把目标角度换算为 STEP 脉冲数。
 *   2. motor.c 根据 Move_Speed 设置 PWM 周期，PWM 硬件连续输出 STEP。
 *   3. PWM_0/PWM_1 的 ZERO 周期中断各自计数：每经过一个 PWM 周期，认为输出了一个 STEP。
 *   4. 计数达到 target_steps 后，停止该轴 PWM 输出，EN 继续保持有效，云台到位锁定。
 *
 * 与旧位置例程的区别：
 *   旧版使用一个 TIMA1 的 CC0/CC1 比较翻转中断产生两路 STEP。
 *   新版使用 SysConfig 生成的两个独立 PWM，只在周期中断里计步，不再手动翻转 STEP。
 */
#include "board.h"
#include "control.h"
#include "tracking.h"
#include "usart.h"
#include "yb_protocol.h"

/* 当前不再通过 Flag_Stop 控制启停。 */
int Flag_Stop = 0;

static volatile uint8_t TrackingKeyAction = KEY_EVENT_NONE;
static char DebugCommand[32];

/*
 * 系统入口。
 *
 * 初始化时会先让两轴进入停止状态，再打开 UART 和 5ms 定时器。
 * 上电不会自动跟踪，必须收到 TRACK 命令或 K230 按键控制帧。
 */
int main(void)
{
    SYSCFG_DL_init();

    /*
     * SysConfig 完成 PWM/GPIO/UART/TIMER 初始化后，应用层只做使能和启动。
     * 不再使用 Motor_Init()，避免和图形化配置重复配置同一个定时器。
     */
    DL_GPIO_setPins(EN_PORT, EN_MA_PIN | EN_MB_PIN);
    DL_TimerA_startCounter(PWM_0_INST);
    DL_TimerA_startCounter(PWM_1_INST);

    /* 上电默认 0 速锁定：STEP 强制低电平，EN 保持有效。 */
    Motor_StopAxis(GIMBAL_AXIS_X);
    Motor_StopAxis(GIMBAL_AXIS_Y);

    /* UART0 接收 K230；UART1 向电脑发送启动和调试信息。 */
    Uart0_Init();
    Debug_Uart_Init();
    Debug_Uart_SendString("tracking_gimbal UART ready\r\n");
    Tracking_Init();

    /* TIMG0 5ms 控制循环，用于按键扫描和 LED 心跳。 */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) {
        /*
         * 主循环固定处理顺序：
         *   1. 解析 K230 数据；
         *   2. 消费 K230 按键控制命令；
         *   3. 更新视觉跟踪；
         *   4. 处理电脑调试命令；
         *   5. 处理 MSPM0 按键事件。
         *
         * 所有耗时字符串处理都放在主循环，UART 和 PWM 中断只做最轻量工作。
         */
        Pto_Loop();

        {
            uint8_t remote_command = Pto_TakeControlCommand();

            if (remote_command == PTO_CONTROL_START) {
                Tracking_Start();
            } else if (remote_command == PTO_CONTROL_STOP) {
                Tracking_Stop();
            }
        }

        Tracking_Loop();

        if (Debug_Uart_GetCommandLine(DebugCommand, sizeof(DebugCommand)) != 0U) {
            Debug_Uart_SendString("CMD RX: ");
            Debug_Uart_SendString(DebugCommand);
            Debug_Uart_SendString("\r\n");
            Tracking_ProcessCommand(DebugCommand);
        }

        if (TrackingKeyAction != KEY_EVENT_NONE) {
            uint8_t action = TrackingKeyAction;

            TrackingKeyAction = KEY_EVENT_NONE;
            if (action == KEY_EVENT_SINGLE) {
                Tracking_Stop();
            } else if (action == KEY_EVENT_DOUBLE) {
                Tracking_Start();
            }
        }

    }
}

/* X轴 PWM 周期中断：每次 ZERO 表示 PWM 走完一个 STEP 周期。 */
void PWM_0_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(PWM_0_INST) == DL_TIMER_IIDX_ZERO) {
        Gimbal_PositionStepISR(GIMBAL_AXIS_X);
    }
}

/* Y轴 PWM 周期中断：逻辑和 X 轴相同，但使用独立定时器。 */
void PWM_1_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(PWM_1_INST) == DL_TIMER_IIDX_ZERO) {
        Gimbal_PositionStepISR(GIMBAL_AXIS_Y);
    }
}

/*
 * 5ms 时基中断：
 *   - 维护跟踪软件时间；
 *   - 刷新 LED；
 *   - 扫描 MSPM0 板载按键。
 * 中断中只置按键事件标志，不在中断里执行命令字符串处理。
 */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        uint8_t key_event;

        LED_Flash(200);        /* 200 * 5ms = 1s LED 心跳。 */
        Tracking_Tick5ms();

        /* 单击停止自动跟踪，双击重新开始。 */
        key_event = click_N_Double(0U);
        if (key_event != KEY_EVENT_NONE) {
            TrackingKeyAction = key_event;
        }

    }
}
