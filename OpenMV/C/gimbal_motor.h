#ifndef GIMBAL_MOTOR_H
#define GIMBAL_MOTOR_H

// 一脉冲 0.05625度
// 角速度 = 0.05625度 * 脉冲频率 
// 脉冲频率 = 角速度 / 0.05625度
// 30角速度：30 / 0.05625 = 533.33Hz

#include <stdint.h>

#define GIMBAL_MOTOR_L 0
#define GIMBAL_MOTOR_R 1

#define GIMBAL_MOTOR_DIRECTION_FORWARD 0
#define GIMBAL_MOTOR_DIRECTION_REVERSE 1

void gimbal_motor_init(uint8_t motor_id);
void gimbal_motor_set_dir(uint8_t motor_id, uint8_t direction);
void gimbal_motor_start(uint8_t motor_id);
void gimbal_motor_stop(uint8_t motor_id);
void gimbal_motor_set_speed(uint8_t motor_id, uint8_t speed);
void gimbal_motor_set_continuous(uint8_t motor_id, uint8_t continuous);
// void step_motor_step_set(uint8_t step, uint8_t stepper_id);
void gimbal_motor_set_angle(uint8_t motor_id, uint8_t angle);

#endif