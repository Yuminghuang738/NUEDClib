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
#include "key.h"
#include "jy61p_port.h"
#include "angle_control.h"

#include "../OpenMV/C/gimbal_motor.h"
#include "../OpenMV/C/vision.h"

volatile int status = 0;
volatile uint32_t sys_tick_ms = 0;

extern u8 OLED_GRAM[144][8];  /* oled.c 显存 */

typedef enum
{
    MODE_HOLD = 0,
    MODE_STRAIGHT = 1,
} drive_mode_t;

static const char *reset_name(uint32_t cause)
{
    switch (cause)
    {
        case 0x04: return "!! BOR !!";
        case 0x02: return "POR NRST";
        case 0x09: return "Boot NRST";
        case 0x00: return "No Reset";
        default:   return "Other";
    }
}

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

    DL_Timer_startCounter(PID_INST);
    NVIC_EnableIRQ(PID_INST_INT_IRQN);

    /* 禁用编码器 GPIO 中断 (旧代码从不使能, ANG 模式不需要编码器计数)
     * 保留按键中断 (KEY_1/KEY_2/KEY_4 在 GPIOB, KEY_3 在 GPIOA) */
    DL_GPIO_disableInterrupt(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(KEY_GPIOA_INT_IRQN );

    // motor_set_direction(MOTOR_L, MOTOR_FORWARD);
    // motor_set_direction(MOTOR_R, MOTOR_FORWARD);

    // gimbal_motor_init(GIMBAL_MOTOR_L);
    // gimbal_motor_init(GIMBAL_MOTOR_R);

    JY61P_Angle angle;
    float  target_angle  = 0.0f;
    float  yaw_offset    = 0.0f;   /* 进入 ANG 模式时的 Yaw, 用于 OLED 归零显示 */
    int    selected_mode  = 1;      /* 预选模式: 1=循迹, 2=角度直走 */
    uint32_t last_oled      = 0;
    uint32_t last_angle_ctrl = 0;

    uint32_t last_debug = 0;

    while (1)
    {
        JY61P_Read_Angle(&angle);

        /* ── Update OLED buffer (IRQ‑protected against M0+ soft‑float) ── */
        NVIC_DisableIRQ(PID_INST_INT_IRQN);
        {
            char buf[24];
            if (status == 2) {
                float yaw_norm = angle.yaw - yaw_offset;
                while (yaw_norm >  180.0f) yaw_norm -= 360.0f;
                while (yaw_norm < -180.0f) yaw_norm += 360.0f;
                float err = target_angle - angle.yaw;
                while (err >  180.0f) err -= 360.0f;
                while (err < -180.0f) err += 360.0f;
                sprintf(buf, "T:  0.0 C:%4.1f", yaw_norm);
                OLED_ShowString(0, 0, (uint8_t *)buf, 12);
                sprintf(buf, "E:%4.1f Kp:%.0f", err, Angle_Kp);
                OLED_ShowString(0, 16, (uint8_t *)buf, 12);
            } else if (status == 1) {
                sprintf(buf, "TRC S:%3.0f", Base_Speed_mm_s);
                OLED_ShowString(0, 0, (uint8_t *)buf, 12);
                sprintf(buf, "L:%3.0f R:%3.0f",
                        Speed_PID_GetActualL(), Speed_PID_GetActualR());
                OLED_ShowString(0, 16, (uint8_t *)buf, 12);
            } else {
                sprintf(buf, "STOP MODE:%s",
                        selected_mode == 1 ? "TRC" : "ANG");
                OLED_ShowString(0, 0, (uint8_t *)buf, 12);
            }
        }
        NVIC_EnableIRQ(PID_INST_INT_IRQN);

        /* ── Angle control at 10ms (status==2 only, disables IRQ for soft‑float) ── */
        if (status == 2 && sys_tick_ms - last_angle_ctrl >= 10) {
            __disable_irq();
            Angle_Control_Update(target_angle, 1500);
            __enable_irq();
            last_angle_ctrl = sys_tick_ms;
        }

        /* ── Interleaved OLED refresh + JY61P polling (keeps UART FIFO from overflowing) ── */
        if (sys_tick_ms - last_oled >= 50) {
            for (int page = 0; page < 8; page++) {
                OLED_WR_Byte(0xB0 + page, OLED_CMD);
                OLED_WR_Byte(0x00, OLED_CMD);
                OLED_WR_Byte(0x10, OLED_CMD);
                for (int col = 0; col < 128; col++) {
                    OLED_WR_Byte(OLED_GRAM[col][page], OLED_DATA);
                    if ((col & 1) == 0) {
                        JY61P_Read_Angle(&angle);
                    }
                }
            }
            last_oled = sys_tick_ms;
        }

        /* ── Key processing ── */
        if (key_mode_trace) {
            key_mode_trace = 0;
            selected_mode = 1;
        }
        if (key_mode_angle) {
            key_mode_angle = 0;
            selected_mode = 2;
        }
        if (key_start_flag) {
            key_start_flag = 0;
            status = selected_mode;
            Speed_PID_Reset();
            if (status == 2) {
                DL_GPIO_disableInterrupt(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);
                yaw_offset    = angle.yaw;
                target_angle  = angle.yaw;
                Angle_PID_Reset();
                angle_ctrl_active = 0;
            }
            if (status == 1) {
                DL_GPIO_clearInterruptStatus(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);
                DL_GPIO_enableInterrupt(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);
                counter_1_A = 0;
                counter_2_A = 0;
            }
        }
        if (key_stop_flag) {
            key_stop_flag = 0;
            status = 0;
            DL_GPIO_disableInterrupt(GPIOB, Motor_l_E1A_PIN | Motor_2_E2A_PIN);
            angle_ctrl_active = 0;
            motor_set_direction(MOTOR_L, MOTOR_STOP);
            motor_set_direction(MOTOR_R, MOTOR_STOP);
            motor_set_duty(MOTOR_L, 0);
            motor_set_duty(MOTOR_R, 0);
        }
    }
}
