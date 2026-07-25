#include "interrupt.h"

uint32_t counter_1_A = 0;
uint32_t counter_2_A = 0;

extern int status;

void GROUP1_IRQHandler(void) 
    {
    switch (DL_GPIO_getPendingInterrupt(GPIOA))  
        {
            case KEY_KEY_3_IIDX:
            {
                status = 2;
                break;
            }
            default:
            break;
        }  
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
        {
        case Motor_l_E1A_IIDX: // 编码器A计数
            {
            counter_1_A ++;
            //DL_GPIO_clearInterruptStatus(GPIOB, Motor_l_E1A_PIN);
            break;
            }

        case Motor_2_E2A_IIDX: // 编码器B计数
            {
            counter_2_A ++;
            //DL_GPIO_clearInterruptStatus(GPIOB,Motor_2_E2A_PIN );
            break;
            }
        case KEY_KEY_1_IIDX:
            {
                status = (status+1) % 3;
                break;
            }
        case KEY_KEY_2_IIDX:
            {
                status = (status+3-1) % 3;
                break;
            }
        case KEY_KEY_4_IIDX:
            {
                status = 0;
                break;
            }
        default:
            break;
        }
        
    }
