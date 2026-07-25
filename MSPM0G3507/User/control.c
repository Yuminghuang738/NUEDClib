#include "control.h"
#include "control_config.h"
#include "trace.h"
#include "motor.h"

#ifdef CONTROL_OPEN_LOOP
#include "control_open.h"
#else
#include "control_closed.h"
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  control_init
 * ═══════════════════════════════════════════════════════════════════════════ */
void control_init(void)
{
    control_variant_init();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  control_update  — called every PID_PERIOD_MS from timer ISR
 *
 *  This is the ONLY function that writes to motor PWM / direction registers.
 *  All mode‑specific duty math lives in control_open.c or control_closed.c.
 *
 *  Three states:
 *    1. active == 0          → cross / end marker → STOP, reset variant
 *    2. inner || active == 4 → straight (inner on line or lost)
 *    3. otherwise             → normal line‑following
 * ═══════════════════════════════════════════════════════════════════════════ */
void control_update(void)
{
    /* ── 1. Read sensors ───────────────────────────────────────────────── */
    trace_read();

    int8_t  error  = trace_get_error();      /* -10 .. +10                   */
    uint8_t active = trace_get_active();     /*  0 ..  4                     */
    bool    inner  = trace_inner_both_line();/* both inner sensors on line   */

    /* ── 2. Cross / end marker → stop ──────────────────────────────────── */
    if (active == 0) {
        motor_set_direction(MOTOR_L, MOTOR_STOP);
        motor_set_direction(MOTOR_R, MOTOR_STOP);
        motor_set_duty(MOTOR_L, 0);
        motor_set_duty(MOTOR_R, 0);
        control_variant_reset();
        return;
    }

    motor_set_direction(MOTOR_L, MOTOR_FORWARD);
    motor_set_direction(MOTOR_R, MOTOR_FORWARD);

    /* ── 3. Inner‑on‑line OR lost line → straight ──────────────────────── */
    if (inner || active == 4) {
        control_straight_duty();
        return;
    }

    /* ── 4. Normal line‑following ──────────────────────────────────────── */
    control_track_duty(error);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PID_INST_IRQHandler  — timer ISR entry (name must match vector table)
 * ═══════════════════════════════════════════════════════════════════════════ */
void PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(PID_INST))
    {
        case DL_TIMER_IIDX_LOAD:
            control_update();
            break;
        default:
            break;
    }
}
