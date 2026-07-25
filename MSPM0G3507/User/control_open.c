#include "control_config.h"

#ifdef CONTROL_OPEN_LOOP

#include "control_open.h"
#include "motor.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Open‑loop variant hooks
 *
 *  duty = STRAIGHT ± error × GAIN   (direct proportional mapping)
 *
 *  No persistent state — each call is independent.
 * ═══════════════════════════════════════════════════════════════════════════ */

void control_variant_init(void)
{
    /* Open‑loop has no persistent state to initialise */
}

void control_variant_reset(void)
{
    /* Nothing to reset */
}

void control_straight_duty(void)
{
    /* inner sensors on line OR line lost → go straight with bias */
    motor_set_duty(MOTOR_L, DUTY_STRAIGHT + DUTY_BIAS);
    motor_set_duty(MOTOR_R, DUTY_STRAIGHT);
}

void control_track_duty(int8_t error)
{
    /* Normal line‑following: error < 0 → line LEFT  → RIGHT turn (L faster)
     *                        error > 0 → line RIGHT → LEFT  turn (R faster) */
    int32_t duty_L = (int32_t)DUTY_STRAIGHT - (int32_t)error * DUTY_GAIN;
    int32_t duty_R = (int32_t)DUTY_STRAIGHT + (int32_t)error * DUTY_GAIN;

    motor_set_duty(MOTOR_L, limit_duty(duty_L));
    motor_set_duty(MOTOR_R, limit_duty(duty_R));
}

#endif /* CONTROL_OPEN_LOOP */
