#include "trace.h"
#include "motor.h"

extern volatile uint8_t angle_ctrl_active;

uint8_t trace_data[4] = {0, 0, 0, 0};

/* Duty cycle constants (PWM period = 4000) */
#define DUTY_MAX       4000
#define DUTY_STRAIGHT  1100
#define DUTY_GAIN       280

uint8_t get_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio);
    if ((gpio & high_bits) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void trace_get_value()
{
    trace_data[0] = get_gpio_state(TRACE_X1_PORT, TRACE_X1_PIN);
    trace_data[1] = get_gpio_state(TRACE_X2_PORT, TRACE_X2_PIN);
    trace_data[2] = get_gpio_state(TRACE_X3_PORT, TRACE_X3_PIN);
    trace_data[3] = get_gpio_state(TRACE_X4_PORT, TRACE_X4_PIN);
}

void trace_motor(void)
{
    /* 角度闭环接管时, 循迹不输出电机控制 */
    if (angle_ctrl_active) return;

    trace_get_value();

    uint8_t  active = trace_data[0] + trace_data[1] + trace_data[2] + trace_data[3];
    int8_t   error  = 0;
    if (trace_data[0]) error += -1;
    if (trace_data[1]) error += -4;
    if (trace_data[2]) error += +1;
    if (trace_data[3]) error += +4;

    static uint8_t was_lost = 0;

    if (active == 0)
    {
        was_lost = 0;
        motor_set_direction(1, 0);
        motor_set_direction(2, 0);
        motor_set_duty(1, 0);
        motor_set_duty(2, 0);
        return;
    }

    if (trace_data[0] + trace_data[2] == 0)
    {
        was_lost = 0;
        motor_set_direction(1, 1);
        motor_set_direction(2,1);
        motor_set_duty(1, DUTY_STRAIGHT+20);
        motor_set_duty(2, DUTY_STRAIGHT);
        return;
    }

    if (active == 4)
    {
        was_lost = 1;
        motor_set_direction(1, 1);
        motor_set_direction(2, 1);
        motor_set_duty(1, DUTY_STRAIGHT - 300);
        motor_set_duty(2, DUTY_STRAIGHT - 280);
        return;
    }
    was_lost = 0;

    motor_set_direction(1, 1);
    motor_set_direction(2, 1);

    int32_t duty_L = (int32_t)DUTY_STRAIGHT - (int32_t)error * DUTY_GAIN;
    int32_t duty_R = (int32_t)DUTY_STRAIGHT + (int32_t)error * DUTY_GAIN;

    if (duty_L < 0)   duty_L = 0;
    if (duty_L > (int32_t)DUTY_MAX) duty_L = DUTY_MAX;
    if (duty_R < 0)   duty_R = 0;
    if (duty_R > (int32_t)DUTY_MAX) duty_R = DUTY_MAX;

    motor_set_duty(1, (uint32_t)duty_L);
    motor_set_duty(2, (uint32_t)duty_R);
}
