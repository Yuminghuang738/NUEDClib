#include "motor.h"
#include "trace.h"

void motor_init(uint8_t motor_id)
{
    if (motor_id == MOTOR_L)
    {
        DL_GPIO_setPins(Motor_l_AIN1_PORT, Motor_l_AIN1_PIN);
        DL_GPIO_setPins(Motor_l_AIN2_PORT, Motor_l_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWMA_INST, 0U, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    }
    else if (motor_id == MOTOR_R)
    {
        DL_GPIO_setPins(Motor_2_PORT, Motor_2_BIN1_PIN);
        DL_GPIO_setPins(Motor_2_PORT, Motor_2_BIN2_PIN);
        DL_TimerA_setCaptureCompareValue(PWMB_INST, 0U, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    }
}

uint32_t limit_duty(int32_t duty)
{
    if (duty > (int32_t)PWM_DUTY_MAX) return PWM_DUTY_MAX;
    if (duty < 0) return 0;
    return (uint32_t)duty;
}

void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    duty = limit_duty((int32_t)duty);

    if (motor_id == MOTOR_L)
    {
        DL_TimerG_setCaptureCompareValue(PWMA_INST, duty, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    }
    else if (motor_id == MOTOR_R)
    {
        DL_TimerA_setCaptureCompareValue(PWMB_INST, duty, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    }
}

void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if (motor_id == MOTOR_L)
    {
        switch (direction)
        {
            case MOTOR_FORWARD:   /* forward  */
                DL_GPIO_setPins(Motor_l_AIN1_PORT, Motor_l_AIN1_PIN);
                DL_GPIO_clearPins(Motor_l_AIN2_PORT, Motor_l_AIN2_PIN);
                break;

            case MOTOR_BACKWARD:   /* backward */
                DL_GPIO_clearPins(Motor_l_AIN1_PORT, Motor_l_AIN1_PIN);
                DL_GPIO_setPins(Motor_l_AIN2_PORT, Motor_l_AIN2_PIN);
                break;

            case MOTOR_STOP:
            default:    /* stop     */
                DL_GPIO_clearPins(Motor_l_AIN1_PORT, Motor_l_AIN1_PIN);
                DL_GPIO_clearPins(Motor_l_AIN2_PORT, Motor_l_AIN2_PIN);
                break;
        }
    }
    else if (motor_id == MOTOR_R)
    {
        switch (direction)
        {
            case MOTOR_FORWARD:   /* forward  */
                DL_GPIO_setPins(Motor_2_PORT, Motor_2_BIN1_PIN);
                DL_GPIO_clearPins(Motor_2_PORT, Motor_2_BIN2_PIN);
                break;

            case MOTOR_BACKWARD:   /* backward */
                DL_GPIO_clearPins(Motor_2_PORT, Motor_2_BIN1_PIN);
                DL_GPIO_setPins(Motor_2_PORT, Motor_2_BIN2_PIN);
                break;

            case MOTOR_STOP:
            default:    /* stop     */
                DL_GPIO_clearPins(Motor_2_PORT, Motor_2_BIN1_PIN);
                DL_GPIO_clearPins(Motor_2_PORT, Motor_2_BIN2_PIN);
                break;
        }
    }
}

uint16_t PID_T = 20;

extern volatile uint32_t counter_1_A;
extern volatile uint32_t counter_2_A;

volatile float speed_1 = 0;
volatile float speed_2 = 0;

void calculate_speed(uint8_t motor_id)
{
    
    if (motor_id == MOTOR_L)
    {
        uint32_t cnt = counter_1_A;
        counter_1_A = 0;
        speed_1 = (float)cnt / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 1000/PID_T;
    }
    if (motor_id == MOTOR_R)
    {
        uint32_t cnt = counter_2_A;
        counter_2_A = 0;
        speed_2 = (float)cnt / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 1000/PID_T;
    }
}

float kp = 0.5;
float ki = 0.4;

#define INTEGRAL_MAX  2000.0f
#define INTEGRAL_MIN -2000.0f

volatile uint16_t PWM_1_duty = 0;
volatile float target_speed_1 = 0;
float last_error_1 = 0;
float current_error_1 = 0;
float integral_1 = 0;

volatile uint16_t PWM_2_duty = 0;
volatile float target_speed_2 = 0;
float last_error_2 = 0;
float current_error_2 = 0;
float integral_2 = 0;

void motor_PID(uint8_t motor_id)
{
    float error;
    if (motor_id == MOTOR_L) 
    {
        error = target_speed_1 - speed_1;
        current_error_1 = error;

        /* 增量式 PID + 积分限幅防饱和 */
        float i_inc = ki * current_error_1;
        integral_1 += i_inc;
        if (integral_1 >  INTEGRAL_MAX) { integral_1 = INTEGRAL_MAX; i_inc = 0; }
        if (integral_1 < -INTEGRAL_MAX) { integral_1 = -INTEGRAL_MAX; i_inc = 0; }

        int32_t delta = (int32_t)(kp * (current_error_1 - last_error_1) + i_inc);
        PWM_1_duty = (uint16_t)limit_duty((int32_t)PWM_1_duty + delta);
        last_error_1 = current_error_1;
        motor_set_duty(motor_id, (uint32_t)PWM_1_duty);
    }
    if (motor_id == MOTOR_R) 
    {
        error = target_speed_2 - speed_2;
        current_error_2 = error;

        /* 增量式 PID + 积分限幅防饱和 */
        float i_inc = ki * current_error_2;
        integral_2 += i_inc;
        if (integral_2 >  INTEGRAL_MAX) { integral_2 = INTEGRAL_MAX; i_inc = 0; }
        if (integral_2 < -INTEGRAL_MAX) { integral_2 = -INTEGRAL_MAX; i_inc = 0; }

        int32_t delta = (int32_t)(kp * (current_error_2 - last_error_2) + i_inc);
        PWM_2_duty = (uint16_t)limit_duty((int32_t)PWM_2_duty + delta);
        last_error_2 = current_error_2;
        motor_set_duty(motor_id, (uint32_t)PWM_2_duty);
    }
}

volatile uint8_t angle_ctrl_active = 0; /* 角度闭环接管标志 */
static volatile uint32_t last_angle_ctrl_ms = 0; /* 上次角度闭环调用时刻 */
extern volatile uint32_t sys_tick_ms;

/*
 * Motor_SetSpeed — 角度闭环电机速度输出接口
 *
 * left  : 左轮速度 (PWM 占空比, 正=前进, 负=后退)
 * right : 右轮速度 (PWM 占空比, 正=前进, 负=后退)
 *
 * 调用后暂停速度 PID, 由角度闭环直接控制电机。
 * 若超过 100ms 未被调用, 自动恢复速度 PID 控制。
 */
void Motor_SetSpeed(int16_t left, int16_t right)
{
    uint16_t duty;

    angle_ctrl_active = 1;
    last_angle_ctrl_ms = sys_tick_ms;

    /* ── 左电机 (motor_id = 1) ── */
    if (left >= 0) {
        motor_set_direction(1, 1);          /* 正转 */
        duty = (uint16_t)left;
    } else {
        motor_set_direction(1, 2);          /* 反转 */
        duty = (uint16_t)(-left);
    }
    motor_set_duty(1, (uint32_t)duty);

    /* ── 右电机 (motor_id = 2) ── */
    if (right >= 0) {
        motor_set_direction(2, 1);          /* 正转 */
        duty = (uint16_t)right;
    } else {
        motor_set_direction(2, 2);          /* 反转 */
        duty = (uint16_t)(-right);
    }
    motor_set_duty(2, (uint32_t)duty);
}

void PID_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
    {
        calculate_speed(1);
        calculate_speed(2);

        /* 角度闭环超时 50ms 未刷新 → 自动切回速度 PID */
        if (angle_ctrl_active && (sys_tick_ms - last_angle_ctrl_ms > 100)) {
            angle_ctrl_active = 0;
            target_speed_1 = speed_1;
            target_speed_2 = speed_2;
        }

        if (!angle_ctrl_active) {
            motor_PID(MOTOR_L);
            motor_PID(MOTOR_R);
        }
        break;
    }
    default:
        break;
    }
}
