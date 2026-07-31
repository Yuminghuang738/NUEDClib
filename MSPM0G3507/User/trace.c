#include "trace.h"

uint8_t trace_data[6] = {0, 0, 0, 0, 0, 0};

/*
 * Sensor layout (left → right):
 *   trace_data[0] = X2  (leftmost)    weight -5
 *   trace_data[1] = X3                weight -3
 *   trace_data[2] = X4                weight  0
 *   trace_data[3] = X5                weight  0
 *   trace_data[4] = X6                weight +3
 *   trace_data[5] = X7  (rightmost)   weight +5
 *
 *   error < 0 → line is LEFT  → need RIGHT turn (motor_L faster)
 *   error > 0 → line is RIGHT → need LEFT  turn (motor_R faster)
 */

static uint8_t get_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio);
    return ((gpio & high_bits) != 0) ? 0 : 1;  /* LOW=黑线→1, HIGH=白→0 */
}

void trace_read(void)
{
    trace_data[0] = get_gpio_state(TRACE_X2_PORT, TRACE_X2_PIN);
    trace_data[1] = get_gpio_state(TRACE_X3_PORT, TRACE_X3_PIN);
    trace_data[2] = get_gpio_state(TRACE_X4_PORT, TRACE_X4_PIN);
    trace_data[3] = get_gpio_state(TRACE_X5_PORT, TRACE_X5_PIN);
    trace_data[4] = get_gpio_state(TRACE_X6_PORT, TRACE_X6_PIN);
    trace_data[5] = get_gpio_state(TRACE_X7_PORT, TRACE_X7_PIN);
}

int8_t trace_get_error(void)
{
    int8_t error = 0;
    if (trace_data[0]) error += -5;   /* X2  leftmost  */
    if (trace_data[1]) error += -1.5;   /* X3            */
    if (trace_data[2]) error += -1;   /* X4  leftmost  */
    if (trace_data[3]) error += +1;   /* X5            */
    if (trace_data[4]) error += +3;   /* X6            */
    if (trace_data[5]) error += +5;   /* X7  rightmost */
    return error;
}

uint8_t trace_get_active(void)
{
    return trace_data[0] + trace_data[1] + trace_data[2]
         + trace_data[3] + trace_data[4] + trace_data[5];
}
