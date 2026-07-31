#include <stdio.h>

#include "ti_msp_dl_config.h"

#include "delay.h"
#include "oled.h"
#include "motor.h"
#include "trace.h"
#include "control.h"
#include "speed_pid.h"
#include "uart.h"
#include "interrupt.h"
#include "uart.h"

volatile int status = 0;
volatile uint32_t sys_tick_ms = 0;
extern volatile uint8_t reverse_brake_cnt;
extern volatile uint8_t stop_armed;
extern volatile uint32_t encoder_total;

void _system_post_cinit(void) { }
void __mpu_init(void) { }

void SysTick_Handler(void)
{
    sys_tick_ms++;
}

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    OLED_Clear();
    delay_ms(100);

    delay_ms(100);

    /* ── JY61P gyroscope init ── */
    OLED_ShowString(0, 16, (u8 *)"JY61P Init", 16);
    OLED_Refresh();

    JY61P_Init();

    /* 等待传感器上电稳定 (JY61P 启动时间 < 200ms, 2 秒充裕) */
    delay_ms(2000);

    /* ── Motor init (after JY61P, matching old proven order) ── */
    motor_init(MOTOR_L);
    motor_init(MOTOR_R);
    control_init();

    /* KEY_1=循迹, KEY_2=任务3, KEY_4=任务5/6(循迹+球), 禁用 KEY_3 */
    DL_GPIO_disableInterrupt(GPIOA, KEY_KEY_3_PIN);

    DL_Timer_startCounter(PID_INST);
    NVIC_EnableIRQ(PID_INST_INT_IRQN);

    /* 禁用编码器 GPIO 中断 (旧代码从不使能, ANG 模式不需要编码器计数)
     * 保留按键中断 (KEY_1/KEY_2/KEY_4 在 GPIOB, KEY_3 在 GPIOA) */
    DL_GPIO_disableInterrupt(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);

    motor_set_direction(MOTOR_L, MOTOR_FORWARD);
    motor_set_direction(MOTOR_R, MOTOR_FORWARD);

    enum { MODE_IDLE, MODE_TRACE, MODE_TASK3, MODE_TRACE_BALL };
    int run_mode = MODE_IDLE;

    /* ── 循迹状态 ── */
    enum { TR_IDLE, WAIT_START, CROSS_SEEN, RUNNING, FINISHED };
    int tr_state = TR_IDLE;
    uint32_t start_time = 0;
    uint32_t lap_time = 0;

    /* ── 任务3 轨迹: O → +50mm → -50mm → 稳定 ── */
    enum { T3_TO_P50, T3_HOLD_P50, T3_TO_N50, T3_HOLD_N50, T3_DONE };
    int t3_state = T3_TO_P50;
    uint32_t t3_hold_start = 0;
    uint32_t t3_start_ms = 0;

    uint32_t last_oled = 0;

    while (1)
    {
        /* ── 空闲: 等待按键 ── */
        if (run_mode == MODE_IDLE)
        {
            if (key_start_flag) {
                key_start_flag = 0;
                run_mode = MODE_TRACE;
                status = 1;
                Speed_PID_Reset();
                DL_GPIO_clearInterruptStatus(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);
                DL_GPIO_enableInterrupt(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);
                counter_1_A = 0;
                counter_2_A = 0;
                encoder_total = 0;
                tr_state = WAIT_START;
                stop_armed = 0;
            }
            if (key_task3_flag) {
                key_task3_flag = 0;
                run_mode = MODE_TASK3;
                status = 4;
                Speed_PID_Reset();
                ball_pid_set_target(50.0f);
                t3_state      = T3_TO_P50;
                t3_hold_start = 0;
                t3_start_ms   = sys_tick_ms;
            }
            if (key_task456_flag) {
                key_task456_flag = 0;
                run_mode = MODE_TRACE_BALL;
                status = 3;
                Speed_PID_Reset();
                ball_pid_init();
                ball_pid_set_target(0.0f);
                DL_GPIO_clearInterruptStatus(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);
                DL_GPIO_enableInterrupt(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);
                counter_1_A = 0;
                counter_2_A = 0;
                encoder_total = 0;
                tr_state = WAIT_START;
                stop_armed = 0;
            }

            trace_read();
            char buf[24];
            sprintf(buf, "D:%d%d%d%d%d%d A:%d",
                    trace_data[0], trace_data[1], trace_data[2],
                    trace_data[3], trace_data[4], trace_data[5],
                    trace_get_active());
            OLED_ShowString(0, 0, (uint8_t *)buf, 16);
            sprintf(buf, "E:%d  ", trace_get_error());
            OLED_ShowString(0, 16, (uint8_t *)buf, 16);
            if (sys_tick_ms - last_oled >= 50) {
                OLED_Refresh();
                last_oled = sys_tick_ms;
            }
            continue;
        }

        /* ═══════════════════════════════════════════════════════════════
         *  循迹模式 (KEY_1) / 循迹+球模式 (KEY_4)
         * ═══════════════════════════════════════════════════════════════ */
        if (run_mode == MODE_TRACE || run_mode == MODE_TRACE_BALL)
        {
            trace_read();
            int is_start = trace_is_cross();
            static uint8_t start_hold = 0;
            if (is_start) { start_hold = 10; }
            else if (start_hold > 0) { start_hold--; is_start = 1; }

            switch (tr_state) {
            case WAIT_START:
                if (is_start) {
                    start_time = sys_tick_ms;
                    tr_state = CROSS_SEEN;
                }
                break;
            case CROSS_SEEN:
                if (!is_start) {
                    tr_state = RUNNING;
                }
                break;
            case RUNNING:
                if (run_mode == MODE_TRACE) {
                    if (status == 0) {   /* ISR 在 control_update 中置零 */
                        lap_time = sys_tick_ms - start_time;
                        tr_state = FINISHED;
                    }
                }
                break;
            case FINISHED:
                break;
            default:
                break;
            }

            char buf[24];
            if (tr_state == FINISHED) {
                sprintf(buf, "TIME:%2d.%02d s       ",
                        (int)(lap_time / 1000), (int)((lap_time % 1000) / 10));
                OLED_ShowString(0, 0, (uint8_t *)buf, 16);
            } else if (tr_state == WAIT_START) {
                OLED_ShowString(0, 0, (uint8_t *)"READY           ", 16);
            } else if (run_mode == MODE_TRACE_BALL) {
                sprintf(buf, "T:%2d.%02d X:%d   ",
                        (int)((sys_tick_ms - start_time) / 1000),
                        (int)(((sys_tick_ms - start_time) % 1000) / 10),
                        ball_pid_get_x());
                OLED_ShowString(0, 0, (uint8_t *)buf, 16);
            } else {
                sprintf(buf, "T:%2d.%02d E:%lu R  ",
                        (int)((sys_tick_ms - start_time) / 1000),
                        (int)(((sys_tick_ms - start_time) % 1000) / 10),
                        encoder_total);
                OLED_ShowString(0, 0, (uint8_t *)buf, 16);
            }
        }

        /* ═══════════════════════════════════════════════════════════════
         *  任务3 模式 (KEY_2): 静止球位置控制 O → +50 → -50
         * ═══════════════════════════════════════════════════════════════ */
        if (run_mode == MODE_TASK3)
        {
            int8_t x = ball_pid_get_x();
            int abs_err;

            switch (t3_state) {
            case T3_TO_P50:
                abs_err = (x > 50) ? (x - 50) : (50 - x);
                if (abs_err <= 10) {
                    if (t3_hold_start == 0)
                        t3_hold_start = sys_tick_ms;
                    else if (sys_tick_ms - t3_hold_start >= 500) {
                        ball_pid_set_target(-50.0f);
                        t3_state = T3_TO_N50;
                        t3_hold_start = 0;
                    }
                } else {
                    t3_hold_start = 0;
                }
                break;
            case T3_TO_N50:
                abs_err = (x > -50) ? (x + 50) : (-50 - x);
                if (abs_err <= 10) {
                    if (t3_hold_start == 0)
                        t3_hold_start = sys_tick_ms;
                    else if (sys_tick_ms - t3_hold_start >= 500) {
                        t3_state = T3_HOLD_N50;
                        t3_hold_start = sys_tick_ms;
                    }
                } else {
                    t3_hold_start = 0;
                }
                break;
            case T3_HOLD_N50:
                t3_state = T3_DONE;
                break;
            case T3_DONE:
                break;
            }

            char buf[24];
            sprintf(buf, "X:%4d T:%2d.%01d",
                    (int)x,
                    (int)((sys_tick_ms - t3_start_ms) / 1000),
                    (int)(((sys_tick_ms - t3_start_ms) % 1000) / 100));
            OLED_ShowString(0, 0, (uint8_t *)buf, 16);

            switch (t3_state) {
            case T3_TO_P50:  sprintf(buf, "TO +50"); break;
            case T3_TO_N50:  sprintf(buf, "TO -50"); break;
            case T3_HOLD_N50: case T3_DONE:
                             sprintf(buf, "AT -50 DONE"); break;
            }
            OLED_ShowString(0, 32, (uint8_t *)buf, 16);
        }

        if (sys_tick_ms - last_oled >= 50) {
            OLED_Refresh();
            last_oled = sys_tick_ms;
        }
    }
}
