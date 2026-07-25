#include "gimbal_motor.h"

#include "ti_msp_dl_config.h"

void gimbal_motor_init(uint8_t motor_id) //云台电机初始化
{
    if (motor_id == GIMBAL_MOTOR_L) // 下面的电机
    {
        DL_GPIO_setPins(GIMBAL_MOTOR_GPIO_L_RST_L_PORT, GIMBAL_MOTOR_GPIO_L_RST_L_PIN);
        DL_GPIO_setPins(GIMBAL_MOTOR_GPIO_L_SLP_L_PORT, GIMBAL_MOTOR_GPIO_L_SLP_L_PIN);
        DL_GPIO_setPins(GIMBAL_MOTOR_GPIO_L_DIR_L_PORT, GIMBAL_MOTOR_GPIO_L_DIR_L_PIN);
        DL_GPIO_setPins(GIMBAL_MOTOR_GPIO_L_DCY_L_PORT, GIMBAL_MOTOR_GPIO_L_DCY_L_PIN);
        NVIC_EnableIRQ(GIMBAL_MOTOR_PWM_L_INST_INT_IRQN);
    }
    if (motor_id == GIMBAL_MOTOR_R) // 上面的电机
    {
        DL_GPIO_setPins(GIMBAL_MOTOR_GPIO_R_RST_R_PORT, GIMBAL_MOTOR_GPIO_R_RST_R_PIN);
        DL_GPIO_setPins(GIMBAL_MOTOR_GPIO_R_SLP_R_PORT, GIMBAL_MOTOR_GPIO_R_SLP_R_PIN);
        DL_GPIO_setPins(GIMBAL_MOTOR_GPIO_R_DIR_R_PORT, GIMBAL_MOTOR_GPIO_R_DIR_R_PIN);
        DL_GPIO_setPins(GIMBAL_MOTOR_GPIO_R_DCY_R_PORT, GIMBAL_MOTOR_GPIO_R_DCY_R_PIN);
        NVIC_EnableIRQ(GIMBAL_MOTOR_PWM_R_INST_INT_IRQN);
    }
}

void gimbal_motor_set_dir(uint8_t motor_id, uint8_t direction) // 设置方向
{
    if (motor_id == GIMBAL_MOTOR_L) // 下面的
    {
        if (direction == GIMBAL_MOTOR_DIRECTION_FORWARD) 
        {
            DL_GPIO_clearPins(GIMBAL_MOTOR_GPIO_L_DIR_L_PORT, GIMBAL_MOTOR_GPIO_L_DIR_L_PIN);
        } 
        if (direction == GIMBAL_MOTOR_DIRECTION_REVERSE)
        {
            DL_GPIO_setPins(GIMBAL_MOTOR_GPIO_L_DIR_L_PORT, GIMBAL_MOTOR_GPIO_L_DIR_L_PIN);
        }
    }
    if (motor_id == GIMBAL_MOTOR_R) // 上面的
    {
        if (direction == GIMBAL_MOTOR_DIRECTION_FORWARD)
        {
            DL_GPIO_setPins(GIMBAL_MOTOR_GPIO_R_DIR_R_PORT, GIMBAL_MOTOR_GPIO_R_DIR_R_PIN);
        }
        if (direction == GIMBAL_MOTOR_DIRECTION_REVERSE)
        {
            DL_GPIO_clearPins(GIMBAL_MOTOR_GPIO_R_DIR_R_PORT, GIMBAL_MOTOR_GPIO_R_DIR_R_PIN);
        }
    }
}

void gimbal_motor_start(uint8_t motor_id) // 电机启动
{
    if (motor_id == GIMBAL_MOTOR_L)
    {
        DL_Timer_startCounter(GIMBAL_MOTOR_PWM_L_INST);
        NVIC_EnableIRQ(GIMBAL_MOTOR_PWM_L_INST_INT_IRQN);
    }
    if (motor_id == GIMBAL_MOTOR_R)
    {
        DL_Timer_startCounter(GIMBAL_MOTOR_PWM_R_INST);
        NVIC_EnableIRQ(GIMBAL_MOTOR_PWM_R_INST_INT_IRQN);
    }
}

void gimbal_motor_stop(uint8_t motor_id) 
{
    if (motor_id == GIMBAL_MOTOR_L)
    {
        DL_Timer_stopCounter(GIMBAL_MOTOR_PWM_L_INST);
    }
    if (motor_id == GIMBAL_MOTOR_R)
    {
        DL_Timer_stopCounter(GIMBAL_MOTOR_PWM_R_INST);
    }
}

// 角速度设置 角度/s
void gimbal_motor_set_speed(uint8_t motor_id, uint8_t speed)
{
    // 根据速度设置PWM频率
    uint32_t frequency = (uint32_t)(speed / 0.05625); // 计算所需的PWM频率
    frequency = frequency > 0 ? frequency : 1;
    // float period_sec = 1.0f / frequency;
    // // 1 / DCC_100_PWM2_INST_CLK_FREQ 
    // // period_sec / (1 / DCC_100_PWM2_INST_CLK_FREQ);
    
    if (motor_id == GIMBAL_MOTOR_L) 
    {
        // 计算定时器溢出值
        uint32_t period = GIMBAL_MOTOR_PWM_L_INST_CLK_FREQ / frequency;
        period = period < 65536 ? period : 65535;
        period = period > 800 ? period : 800; 
        DL_Timer_setLoadValue(GIMBAL_MOTOR_PWM_L_INST, period);
        DL_Timer_setCaptureCompareValue(GIMBAL_MOTOR_PWM_L_INST, period / 2, GPIO_GIMBAL_MOTOR_PWM_L_C0_IDX); // 设置占空比为50%
    }
    if (motor_id == GIMBAL_MOTOR_R)
    {
        // 计算定时器溢出值
        uint32_t period = GIMBAL_MOTOR_PWM_R_INST_CLK_FREQ / frequency;
        period = period < 65536 ? period : 65535;
        period = period > 800 ? period : 800; 
        DL_Timer_setLoadValue(GIMBAL_MOTOR_PWM_R_INST, period);
        DL_Timer_setCaptureCompareValue(GIMBAL_MOTOR_PWM_R_INST, period / 2, GPIO_GIMBAL_MOTOR_PWM_R_C0_IDX); // 设置占空比为50%
    }
}

uint32_t step_remain_r = 0;
uint32_t step_remain_l = 0;
static uint8_t gimbal_continuous_r = 0;
static uint8_t gimbal_continuous_l = 0;


void gimbal_motor_set_angle(uint8_t motor_id, uint8_t angle) // 设置要转的度数
{
    if (motor_id == GIMBAL_MOTOR_L)
    {
        // 根据角度设置步数
        step_remain_l = (uint32_t)(angle / 0.05625); // 计算所需的步数
        gimbal_motor_start(motor_id);
    }

    if (motor_id == GIMBAL_MOTOR_R)
    {
        // 根据角度设置步数
        step_remain_r = (uint32_t)(angle / 0.05625); // 计算所需的步数
        gimbal_motor_start(motor_id);
    }

}

void gimbal_motor_set_continuous(uint8_t motor_id, uint8_t continuous)
{
    if (motor_id == GIMBAL_MOTOR_L)
        gimbal_continuous_l = continuous;
    if (motor_id == GIMBAL_MOTOR_R)
        gimbal_continuous_r = continuous;
}

void GIMBAL_MOTOR_PWM_L_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(GIMBAL_MOTOR_PWM_L_INST))
    {
        case DL_TIMER_IIDX_LOAD:
        {
            // 连续模式下不计数，由调用方通过 gimbal_motor_stop 控制停机
            if (gimbal_continuous_l) break;

            if (step_remain_l == 0)
            {
                gimbal_motor_stop(GIMBAL_MOTOR_L);
                break;
            }
            step_remain_l--;
            break;
        }

        default: break;
    }
}

void GIMBAL_MOTOR_PWM_R_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(GIMBAL_MOTOR_PWM_R_INST))
    {
        case DL_TIMER_IIDX_LOAD:
        {
            // 连续模式下不计数，由调用方通过 gimbal_motor_stop 控制停机
            if (gimbal_continuous_r) break;

            if (step_remain_r == 0)
            {
                gimbal_motor_stop(GIMBAL_MOTOR_R);
                break;
            }
            step_remain_r--;
            break;
        }

        default: break;
    }
}