#ifndef CONTROL_OPEN_H
#define CONTROL_OPEN_H

#include <stdint.h>

/* ── Open‑loop constants ── */
#define DUTY_STRAIGHT    1300    /* straight-line base duty                  */
#define DUTY_GAIN         300    /* error-to-duty gain (PWM per error unit)  */
#define DUTY_BIAS          20    /* left-motor bias to compensate asymmetry  */

/* ── Open‑loop hooks ── */
void control_variant_init(void);
void control_variant_reset(void);
void control_straight_duty(void);
void control_track_duty(int8_t error);

#endif
