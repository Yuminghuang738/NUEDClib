#include "key.h"

volatile uint32_t counter_1_A = 0;
volatile uint32_t counter_2_A = 0;

void GROUP1_IRQHandler(void) // 编码器计数
    {
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
        {
        case Motor_l_E1A_IIDX:
            {
            counter_1_A++;
            DL_GPIO_clearInterruptStatus(GPIOB, Motor_l_E1A_PIN);
            break;
            }

        case Motor_2_E2A_IIDX:
            {
            counter_2_A++;
            DL_GPIO_clearInterruptStatus(GPIOB, Motor_2_E2A_PIN);
            break;
            }
        
        default:
            break;
        }
        
    }
