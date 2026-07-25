#include <stdio.h>

#include "ti_msp_dl_config.h"

#include "delay.h"
#include "oled.h"
#include "motor.h"
#include "trace.h"
#include "jy61p_port.h"
#include "interrupt.h"
#include "key.h"

#include "../OpenMV/C/gimbal_motor.h"
#include "../OpenMV/C/vision.h"
#include "angle_control.h"

volatile int status = 0;

volatile uint32_t sys_tick_ms = 0;

typedef enum 
{
    MODE_HOLD = 0,
    MODE_STRAIGHT = 1,
} drive_mode_t;

extern u8 OLED_GRAM[144][8];  /* oled.c 显存 */

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

    uint32_t cause = (uint32_t)DL_SYSCTL_getResetCause();
    char buf[20];
    sprintf(buf, "RST:%s", reset_name(cause));
    OLED_ShowString(0, 0, (u8 *)buf, 16);
    OLED_Refresh();
   
    delay_ms(500);

    OLED_ShowString(0, 16, (u8 *)"JY61P Init", 16);
    OLED_Refresh();

    JY61P_Init();

    /* 等待传感器上电稳定 (JY61P 启动时间 < 200ms, 2 秒充裕) */
    delay_ms(2000);

    motor_init(1);
    motor_init(2);
    motor_set_direction(1, 1);
    motor_set_direction(2, 1);

    // gimbal_motor_init(GIMBAL_MOTOR_L);
    // gimbal_motor_init(GIMBAL_MOTOR_R);

    JY61P_Angle angle;
    char oled_str[48];
    uint32_t last_oled = 0;
    uint32_t last_angle_ctrl = 0;

    /* 启动时捕获当前 Yaw 作为目标角, 默认占空比 2000 */
    JY61P_Read_Angle(&angle);
    float target_angle = angle.yaw;
    int16_t base_speed = 0;    /* 原地静止模式: base=0, 纯 PID 纠偏差速旋转 */

    drive_mode_t drive_mode = MODE_HOLD;
    uint32_t mode_start_ms = sys_tick_ms;

    
    while (1)
    {
        JY61P_Read_Angle(&angle);

        /* 更新显存 */
        sprintf(oled_str, "Roll :%7.1f", angle.roll);
        OLED_ShowString(0, 0,  (u8 *)oled_str, 16);
        sprintf(oled_str, "Pitch:%7.1f", angle.pitch);
        OLED_ShowString(0, 16, (u8 *)oled_str, 16);
        sprintf(oled_str, "Yaw  :%7.1f", angle.yaw);
        OLED_ShowString(0, 32, (u8 *)oled_str, 16);
        sprintf(oled_str, "T:%3.0f %s D:%4d",
                target_angle,
                (drive_mode == MODE_HOLD) ? "HLD" : "GO>",
                base_speed);
        OLED_ShowString(0, 48, (u8 *)oled_str, 16);

        /* 模式切换: 原地保持 1 秒后自动进入直线行驶 */
        if (drive_mode == MODE_HOLD && (sys_tick_ms - mode_start_ms >= 1000)) {
            drive_mode = MODE_STRAIGHT;
            base_speed = 1500;
            Angle_PID_Reset();
        }

        /* 角度闭环控制: 10ms 周期调用, 关中断保护软浮点 */
        if (sys_tick_ms - last_angle_ctrl >= 10) {
            __disable_irq();
            Angle_Control_Update(target_angle, base_speed);
            __enable_irq();
            last_angle_ctrl = sys_tick_ms;
        }

        /* 逐列刷新 OLED，每 2 列读一次陀螺仪（~560µs），FIFO 不溢出 */
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
    }
}
