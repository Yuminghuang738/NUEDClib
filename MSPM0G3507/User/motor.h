#ifndef MOTOR_H
#define MOTOR_H

#define PI 3.1415926f

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ── Motor hardware parameters ── */
#define MOTOR_BIANMAQI   260     /* encoder pulses per revolution */
#define MOTOR_WHEEL_D     44     /* wheel diameter, mm */

/* ── PWM duty limits ── */
#define PWM_PERIOD      4000     /* timer count period */
#define PWM_DUTY_MAX    4000

#define MOTOR_L 1
#define MOTOR_R 2

#define MOTOR_FORWARD  1
#define MOTOR_BACKWARD 2
#define MOTOR_STOP     0

void     motor_init(uint8_t motor_id);
void     motor_set_duty(uint8_t motor_id, uint32_t duty);
void     motor_set_direction(uint8_t motor_id, uint8_t direction);
uint32_t limit_duty(int32_t duty);

/* ── Encoder (called from control ISR) ── */
int32_t  motor_read_encoder(uint8_t motor_id);  /* returns speed Q8.8 (mm/s × 256) */

extern volatile uint32_t sys_tick_ms;

/* ── Angle control takeover (called from angle_control.c) ── */
extern volatile uint8_t  angle_ctrl_active;
extern volatile uint32_t last_angle_ctrl_ms;
void Motor_SetSpeed(int16_t left, int16_t right);

#endif
