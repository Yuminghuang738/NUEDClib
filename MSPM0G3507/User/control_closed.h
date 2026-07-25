#ifndef CONTROL_CLOSED_H
#define CONTROL_CLOSED_H

#include <stdint.h>

/* ── Closed‑loop constants (Q8.8 fixed‑point) ── */
#define BASE_SPEED_Q8_8   25600    /* 100.0 mm/s × 256                       */
#define SPEED_GAIN_Q8_8    2560    /*  10.0 × 256                            */
#define KP_Q8_8            128     /*   0.5 × 256                            */
#define KI_Q8_8            102     /*   0.4 × 256                            */

/* ── Closed‑loop hooks ── */
void control_variant_init(void);
void control_variant_reset(void);
void control_straight_duty(void);
void control_track_duty(int8_t error);

#endif
