#include "interrupt.h"

volatile uint32_t counter_1_A = 0;
volatile uint32_t counter_2_A = 0;

volatile uint8_t key_start_flag  = 0;
volatile uint8_t key_mode_trace   = 0;
volatile uint8_t key_mode_angle   = 0;
volatile uint8_t key_stop_flag    = 0;

extern volatile int status;

void GROUP1_IRQHandler(void)
{
    uint32_t iidx;

    /* 处理 GPIOA 全部挂起源 */
    while ((iidx = DL_GPIO_getPendingInterrupt(GPIOA)) != 0) {
        switch (iidx) {
            case KEY_KEY_3_IIDX:
                key_mode_angle = 1;   /* 选定角度模式 */
                break;
            default:
                break;
        }
        DL_GPIO_clearInterruptStatus(GPIOA, (1UL << iidx));
    }

    /* 处理 GPIOB 全部挂起源 */
    while ((iidx = DL_GPIO_getPendingInterrupt(GPIOB)) != 0) {
        switch (iidx) {
            case Motor_l_E1A_IIDX:
                counter_1_A++;
                break;
            case Motor_2_E2A_IIDX:
                counter_2_A++;
                break;
            case KEY_KEY_1_IIDX:
                key_start_flag = 1;   /* 发车 */
                break;
            case KEY_KEY_2_IIDX:
                key_mode_trace = 1;   /* 选定循迹模式 */
                break;
            case KEY_KEY_4_IIDX:
                key_stop_flag = 1;    /* 停车 */
                break;
            default:
                break;
        }
        DL_GPIO_clearInterruptStatus(GPIOB, (1UL << iidx));
    }
}
