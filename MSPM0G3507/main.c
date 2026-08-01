#include <stdio.h>

#include "ti_msp_dl_config.h"

#include "delay.h"
#include "oled.h"
#include "motor.h"
#include "trace.h"
#include "control.h"
#include "speed_pid.h"
#include "interrupt.h"
#include "../OpenMV/C/vision.h"
#include "../OpenMV/C/gimbal_motor.h"

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

    motor_init(MOTOR_L);
    motor_init(MOTOR_R);
    control_init();
    gimbal_motor_init(GIMBAL_MOTOR_L);
    gimbal_motor_init(GIMBAL_MOTOR_R);

    /* KEY_1=循迹停车, KEY_2=任务三, KEY_3=任务三, KEY_4=循迹+平衡 */
    DL_GPIO_enableInterrupt(GPIOA, KEY_KEY_3_PIN);
    NVIC_EnableIRQ(KEY_GPIOA_INT_IRQN);

    DL_Timer_startCounter(PID_INST);
    NVIC_EnableIRQ(PID_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);

    motor_set_direction(MOTOR_L, MOTOR_FORWARD);
    motor_set_direction(MOTOR_R, MOTOR_FORWARD);

    enum { MODE_IDLE, MODE_TRACE, MODE_VISION };
    int run_mode = MODE_IDLE;
    bool task3_active = false;
    bool balance_enabled = false;

    /* ── 循迹状态 ── */
    enum { TR_IDLE, WAIT_START, CROSS_SEEN, RUNNING, FINISHED };
    int tr_state = TR_IDLE;
    uint32_t start_time = 0;
    uint32_t lap_time = 0;

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
                nostop_mode = 0;
                Speed_PID_Reset();
                DL_GPIO_clearInterruptStatus(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);
                DL_GPIO_enableInterrupt(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);
                counter_1_A = 0;
                counter_2_A = 0;
                encoder_total = 0;
                tr_state = WAIT_START;
                stop_armed = 0;
            }
            if (key_nostop_flag) {
                key_nostop_flag = 0;
                run_mode = MODE_VISION;
                task3_active = true;
                status = 4;
                gimbal_motor_stop(GIMBAL_MOTOR_R);
            }
            if (key_vision_flag) {
                key_vision_flag = 0;
                run_mode = MODE_TRACE;
                status = 1;
                nostop_mode = 1;
                balance_enabled = 1;
                Speed_PID_Reset();
                DL_GPIO_clearInterruptStatus(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);
                DL_GPIO_enableInterrupt(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);
                counter_1_A = 0;
                counter_2_A = 0;
                encoder_total = 0;
                tr_state = WAIT_START;
                stop_armed = 0;
            }
            if (key_angle_flag) {
                key_angle_flag = 0;
                run_mode = MODE_VISION;
                task3_active = true;
                status = 4;
                gimbal_motor_stop(GIMBAL_MOTOR_R);
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
         *  循迹模式 (KEY_1 停车 / KEY_4 不停车)
         * ═══════════════════════════════════════════════════════════════ */
        if (run_mode == MODE_TRACE)
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
                if (!nostop_mode) {
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

            /* KEY_4: 停止循迹+平衡, 返回 IDLE */
            if (key_vision_flag) {
                key_vision_flag = 0;
                status = 0;
                nostop_mode = 0;
                balance_enabled = 0;
                gimbal_motor_stop(GIMBAL_MOTOR_R);
                run_mode = MODE_IDLE;
                continue;
            }

            /* 不停车模式运行平衡 */
            if (balance_enabled)
                process_deviation();

            char buf[24];
            if (tr_state == FINISHED) {
                sprintf(buf, "TIME:%2d.%02d s       ",
                        (int)(lap_time / 1000), (int)((lap_time % 1000) / 10));
                OLED_ShowString(0, 0, (uint8_t *)buf, 16);
            } else if (tr_state == WAIT_START) {
                OLED_ShowString(0, 0, (uint8_t *)"READY           ", 16);
            } else {
                sprintf(buf, "T:%2d.%02d E:%lu %s%s",
                        (int)((sys_tick_ms - start_time) / 1000),
                        (int)(((sys_tick_ms - start_time) % 1000) / 10),
                        encoder_total,
                        nostop_mode ? "NS" : "R ",
                        balance_enabled ? " BAL" : "");
                OLED_ShowString(0, 0, (uint8_t *)buf, 16);
            }
        }

        /* ═══════════════════════════════════════════════════════
         *  视觉模式 — 任务三 / 视觉平衡
         * ═══════════════════════════════════════════════════════ */
        if (run_mode == MODE_VISION)
        {
            if (task3_active)
                process_deviation_task3();
            else
                process_deviation();

            char buf[24];
            if (task3_active) {
                /* 调试: S=状态 D=期望角 C=当前角 →=方向 */
                int d = (int)((dbg_t3_desired + 128) >> 8);
                int c = (int)((dbg_t3_current + 128) >> 8);
                char dir_c = (dbg_t3_dir > 0) ? 'F' : ((dbg_t3_dir < 0) ? 'R' : 'S');
                sprintf(buf, "S%d D:%+3d C:%+3d %c",
                        dbg_t3_state, d, c, dir_c);
                OLED_ShowString(0, 0, (uint8_t *)buf, 16);
            } else {
                sprintf(buf, "VISION BAL");
                OLED_ShowString(0, 0, (uint8_t *)buf, 16);
            }
        }

        if (sys_tick_ms - last_oled >= 50) {
            OLED_Refresh();
            last_oled = sys_tick_ms;
        }
    }
}
