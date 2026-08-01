#include "interrupt.h"

volatile uint32_t counter_1_A = 0;
volatile uint32_t counter_2_A = 0;
volatile uint32_t encoder_total = 0;   /* 累计编码脉冲, 停止里程门禁 */
volatile uint8_t  key_start_flag  = 0;
volatile uint8_t  key_nostop_flag = 0;
volatile uint8_t  key_angle_flag  = 0;  /* KEY_2: MODE_VISION 内切换任务三 */
volatile uint8_t  key_vision_flag = 0;  /* KEY_4: 进入/退出视觉模式 */
volatile uint8_t  nostop_mode     = 0;

void GROUP1_IRQHandler(void)
{
    uint32_t iidx;

    /* GPIOA: KEY_3 */
    while ((iidx = DL_GPIO_getPendingInterrupt(GPIOA)) != 0) {
        switch (iidx) {
            case KEY_KEY_3_IIDX:
                key_nostop_flag = 1;
                break;
            default:
                break;
        }
        DL_GPIO_clearInterruptStatus(GPIOA, (1UL << iidx));
    }

    /* GPIOB: 编码器 + KEY_1/KEY_2/KEY_4 */
    while ((iidx = DL_GPIO_getPendingInterrupt(GPIOB)) != 0) {
        switch (iidx) {
            case Motor_l_E1A_IIDX:
                counter_1_A++;
                break;
            case Motor_2_E2A_IIDX:
                counter_2_A++;
                break;
            case KEY_KEY_1_IIDX:
                key_start_flag = 1;
                break;
            case KEY_KEY_2_IIDX:
                key_angle_flag = 1;
                break;
            case KEY_KEY_4_IIDX:
                key_vision_flag = 1;
                break;
            default:
                break;
        }
        DL_GPIO_clearInterruptStatus(GPIOB, (1UL << iidx));
    }
}
