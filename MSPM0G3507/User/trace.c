#include "trace.h"

/* ── Raw sensor readings (0 = line / black, 1 = white background) ── */
uint8_t trace_data[4] = {0, 0, 0, 0};

/*
 * Sensor layout (left → right):
 *   trace_data[1] = X2  (leftmost,  outer)
 *   trace_data[0] = X1  (inner)
 *   trace_data[2] = X3  (inner)
 *   trace_data[3] = X4  (rightmost, outer)
 *
 * Weights for centre‑of‑mass error:
 *   X2: -4    X1: -1    X3: +1    X4: +4
 *
 *   error < 0 → line is LEFT  → need RIGHT turn (motor_L faster)
 *   error > 0 → line is RIGHT → need LEFT  turn (motor_R faster)
 */

/**
 * @brief  Read a single GPIO pin.
 * @return 1 = white (HIGH), 0 = black / line (LOW)
 */
static uint8_t get_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio);
    return ((gpio & high_bits) != 0) ? 1 : 0;
}

/* ── Public API ────────────────────────────────────────────────────────── */

void trace_read(void)
{
    trace_data[0] = get_gpio_state(TRACE_X1_PORT, TRACE_X1_PIN);
    trace_data[1] = get_gpio_state(TRACE_X2_PORT, TRACE_X2_PIN);
    trace_data[2] = get_gpio_state(TRACE_X3_PORT, TRACE_X3_PIN);
    trace_data[3] = get_gpio_state(TRACE_X4_PORT, TRACE_X4_PIN);
}

int8_t trace_get_error(void)
{
    int8_t error = 0;
    if (trace_data[0]) error += -1;
    if (trace_data[1]) error += -4;
    if (trace_data[2]) error += +1;
    if (trace_data[3]) error += +4;
    return error;
}

uint8_t trace_get_active(void)
{
    return trace_data[0] + trace_data[1] + trace_data[2] + trace_data[3];
}

bool trace_inner_both_line(void)
{
    /* Both X1 (index 0) and X3 (index 2) see the black line (0) */
    return (trace_data[0] == 0) && (trace_data[2] == 0);
}
