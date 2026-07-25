#include "motor.h"
#include "interrupt.h"

/* ── Speed conversion factor (Q8.8) ────────────────────────────────────────
   speed = counter / MOTOR_BIANMAQI * π * MOTOR_WHEEL_D * 1000 / PID_PERIOD_MS
         = counter * (π * 44 * 50) / 260
         ≈ counter * 26.5827
   Q8.8:  speed_Q8_8 = counter × 26.5827 × 256 ≈ counter × 6805                */
#define SPEED_FACTOR_Q8_8  6805

void motor_init(uint8_t motor_id)
{
    if (motor_id == MOTOR_L)
    {
        /* PWM timer already started by SYSCFG_DL_init();
         * only set initial direction (brake) and zero duty */
        DL_GPIO_setPins(Motor_l_AIN1_PORT, Motor_l_AIN1_PIN);
        DL_GPIO_setPins(Motor_l_AIN2_PORT, Motor_l_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWMA_INST, 0U, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    }
    else if (motor_id == MOTOR_R)
    {
        /* PWM timer already started by SYSCFG_DL_init() */
        DL_GPIO_setPins(Motor_2_PORT, Motor_2_BIN1_PIN);
        DL_GPIO_setPins(Motor_2_PORT, Motor_2_BIN2_PIN);
        DL_TimerA_setCaptureCompareValue(PWMB_INST, 0U, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    }
}

/*
 * QEI encoder mode uses TIMG5/TIMG6 hardware with built-in
 * digital glitch filtering.  No GPIO interrupts needed — the
 * timer hardware counts encoder pulses and filters noise
 * autonomously in the background.
 */

/**
 * @brief  Clamp PWM duty to valid range [0, PWM_DUTY_MAX].
 *         Accepts int32_t so negative-out-of-range is caught at compile time.
 */
uint32_t limit_duty(int32_t duty)
{
    if (duty > (int32_t)PWM_DUTY_MAX)
    {
        return PWM_DUTY_MAX;
    }
    if (duty < 0)
    {
        return 0;
    }
    return (uint32_t)duty;
}

void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    duty = limit_duty((int32_t)duty);

    if (motor_id == MOTOR_L)
    {
        /* PWMA = TIMG7 (general-purpose timer) */
        DL_TimerG_setCaptureCompareValue(PWMA_INST, duty, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    }
    else if (motor_id == MOTOR_R)
    {
        /* PWMB = TIMA0 (advanced timer — different register layout!) */
        DL_TimerA_setCaptureCompareValue(PWMB_INST, duty, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    }
}

/* direction: 0 = stop, 1 = forward, 2 = backward */
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

/**
 * @brief  Read encoder pulse count and reset for next period.
 * @return Speed in Q8.8 fixed-point (mm/s × 256).
 * @note   Called once per motor per control cycle from ISR.
 *         M0+ has no atomic 32-bit read-modify-write — the tiny window
 *         between read and clear may drop at most 1 encoder pulse
 *         (~0.05 mm travel), which is acceptable.
 */
int32_t motor_read_encoder(uint8_t motor_id)
{
    uint32_t count;

    if (motor_id == MOTOR_L)
    {
        count = counter_1_A;
        counter_1_A = 0;
    }
    else  /* MOTOR_R */
    {
        count = counter_2_A;
        counter_2_A = 0;
    }

    return (int32_t)(count * SPEED_FACTOR_Q8_8);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Motor_SetSpeed — angle control output interface
 *
 *  Called from angle_control.c (main loop context).
 *  Sets angle_ctrl_active flag → control_update() ISR skips line‑following.
 *  If not called for >100ms, ISR auto‑recovers to line‑following.
 * ═══════════════════════════════════════════════════════════════════════════ */
volatile uint8_t angle_ctrl_active = 0;
volatile uint32_t last_angle_ctrl_ms = 0;

void Motor_SetSpeed(int16_t left, int16_t right)
{
    uint16_t duty;

    angle_ctrl_active   = 1;
    last_angle_ctrl_ms  = sys_tick_ms;

    /* ── Left motor ── */
    if (left >= 0) {
        motor_set_direction(MOTOR_L, MOTOR_FORWARD);
        duty = (uint16_t)left;
    } else {
        motor_set_direction(MOTOR_L, MOTOR_BACKWARD);
        duty = (uint16_t)(-left);
    }
    motor_set_duty(MOTOR_L, (uint32_t)duty);

    /* ── Right motor ── */
    if (right >= 0) {
        motor_set_direction(MOTOR_R, MOTOR_FORWARD);
        duty = (uint16_t)right;
    } else {
        motor_set_direction(MOTOR_R, MOTOR_BACKWARD);
        duty = (uint16_t)(-right);
    }
    motor_set_duty(MOTOR_R, (uint32_t)duty);
}
