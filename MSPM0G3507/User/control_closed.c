#include "control_config.h"

#ifdef CONTROL_CLOSED_LOOP

#include "control_closed.h"
#include "motor.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Fixed-point conventions (Cortex-M0+ — no hardware FPU)
 *
 *  Speed:      Q8.8  (mm/s × 256)
 *  PID gains:  Q8.8  (dimensionless × 256)
 *  Duty:       raw PWM counts 0..PWM_DUTY_MAX, accumulated in Q8.8 internally
 *
 *  All arithmetic uses int32_t multiply + shift; zero soft-float calls.
 * ═══════════════════════════════════════════════════════════════════════════ */

#define PID_PERIOD_MS         20

#define PWM_DUTY_MAX_Q8_8     ((int32_t)PWM_DUTY_MAX << 8)   /* 4000 × 256 = 1 024 000 */
#define INIT_DUTY_Q8_8        ((int32_t)1300 << 8)           /* smooth-start duty       */

/* ═══════════════════════════════════════════════════════════════════════════
 *  PID (incremental, fixed‑point)
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    int32_t kp;          /* × 256                                            */
    int32_t ki;          /* × 256                                            */
    int32_t last_error;  /* Q8.8                                             */
} pid_t;

static pid_t pid_L;
static pid_t pid_R;

/* Accumulated duty (Q8.8) — the "integral" of the incremental PID.
 * Clamped to [0, PWM_DUTY_MAX_Q8_8] after each update.                     */
static int32_t duty_L_Q8_8;
static int32_t duty_R_Q8_8;

static void pid_reset(pid_t *pid)
{
    pid->last_error = 0;
}

static void pid_init(pid_t *pid, int32_t kp, int32_t ki)
{
    pid->kp = kp;
    pid->ki = ki;
    pid_reset(pid);
}

/* ── Incremental PID step ──────────────────────────────────────────────────
 * Returns Δduty in Q8.8.  Caller accumulates into duty_L_Q8_8 / duty_R_Q8_8
 * and clamps to [0, PWM_DUTY_MAX_Q8_8].
 *
 *   Δout = kp * (e[k] - e[k-1])  +  ki * e[k]
 *
 * All terms Q8.8; products are Q16.16 → arithmetic right-shift 8 → Q8.8.   */
static int32_t pid_update(pid_t *pid, int32_t target, int32_t actual)
{
    int32_t error       = target - actual;                     /* Q8.8       */
    int32_t delta_error = error - pid->last_error;             /* Q8.8       */

    /* kp (×256) × delta_error (Q8.8) → Q16.16
     * ki (×256) × error       (Q8.8) → Q16.16                              */
    int32_t delta = pid->kp * delta_error;                     /* Q16.16     */
    delta        += pid->ki * error;                           /* Q16.16     */

    pid->last_error = error;

    return delta >> 8;                                        /* → Q8.8     */
}

/* ── Helper: clamp accumulators and write duty to hardware ───────────────── */
static void apply_duty(void)
{
    if (duty_L_Q8_8 < 0)                duty_L_Q8_8 = 0;
    if (duty_L_Q8_8 > PWM_DUTY_MAX_Q8_8) duty_L_Q8_8 = PWM_DUTY_MAX_Q8_8;
    if (duty_R_Q8_8 < 0)                duty_R_Q8_8 = 0;
    if (duty_R_Q8_8 > PWM_DUTY_MAX_Q8_8) duty_R_Q8_8 = PWM_DUTY_MAX_Q8_8;

    motor_set_duty(MOTOR_L, (uint32_t)(duty_L_Q8_8 >> 8));
    motor_set_duty(MOTOR_R, (uint32_t)(duty_R_Q8_8 >> 8));
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Variant hooks
 * ═══════════════════════════════════════════════════════════════════════════ */

void control_variant_init(void)
{
    pid_init(&pid_L, KP_Q8_8, KI_Q8_8);
    pid_init(&pid_R, KP_Q8_8, KI_Q8_8);
    duty_L_Q8_8 = INIT_DUTY_Q8_8;
    duty_R_Q8_8 = INIT_DUTY_Q8_8;
}

void control_variant_reset(void)
{
    pid_reset(&pid_L);
    pid_reset(&pid_R);
    duty_L_Q8_8 = 0;
    duty_R_Q8_8 = 0;
}

void control_straight_duty(void)
{
    /* inner sensors on line OR line lost → hold base speed on both wheels   */
    int32_t speed_L = motor_read_encoder(MOTOR_L);
    int32_t speed_R = motor_read_encoder(MOTOR_R);

    int32_t delta_L = pid_update(&pid_L, BASE_SPEED_Q8_8, speed_L);
    int32_t delta_R = pid_update(&pid_R, BASE_SPEED_Q8_8, speed_R);

    duty_L_Q8_8 += delta_L;
    duty_R_Q8_8 += delta_R;

    apply_duty();
}

void control_track_duty(int8_t error)
{
    /* error < 0 → line LEFT  → target L faster
     * error > 0 → line RIGHT → target R faster                              */
    int32_t target_L = BASE_SPEED_Q8_8 - (int32_t)error * SPEED_GAIN_Q8_8;
    int32_t target_R = BASE_SPEED_Q8_8 + (int32_t)error * SPEED_GAIN_Q8_8;

    int32_t speed_L = motor_read_encoder(MOTOR_L);
    int32_t speed_R = motor_read_encoder(MOTOR_R);

    int32_t delta_L = pid_update(&pid_L, target_L, speed_L);
    int32_t delta_R = pid_update(&pid_R, target_R, speed_R);

    duty_L_Q8_8 += delta_L;
    duty_R_Q8_8 += delta_R;

    apply_duty();
}

#endif /* CONTROL_CLOSED_LOOP */
