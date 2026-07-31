#include "speed_pid.h"
#include "motor.h"
#include "oled.h"
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Speed PID — 前馈 + 反馈 (Cortex‑M0+, 无硬件 FPU)
 *
 *  duty = feedforward(target) + PID(error)
 *
 *  前馈:  根据目标速度估算基础占空比, PID 无需从零积分爬升
 *  反馈:  位置式 PID 消除稳态误差和扰动
 *  周期:  20ms (PID_INST_IRQHandler)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── 默认参数 ── */
float Speed_Kp        = 2.0f;    /* 速度 P 增益 (duty / mm/s)       */
float Speed_Ki        = 1.0f;    /* 速度 I 增益 (duty / mm/s·s)     */
float Speed_Kd        = 0.0f;    /* 速度 D 增益 (一般不用)          */
float Base_Speed_mm_s = 280.0f;  /* 基础线速度 mm/s                */
float Max_Speed_mm_s  = 700.0f;  /* 满占空比(4000)时的线速度 mm/s  */

/* ── 内部状态 ── */
static float integral_l    = 0.0f;
static float integral_r    = 0.0f;
static float last_error_l  = 0.0f;
static float last_error_r  = 0.0f;
static volatile float actual_l_mm_s = 0.0f;
static volatile float actual_r_mm_s = 0.0f;

#define SPEED_INTEGRAL_MAX  300.0f
#define SPEED_PID_DT        0.02f

/* ═══════════════════════════════════════════════════════════════════════════
 *  Speed_PID_Init / Reset
 * ═══════════════════════════════════════════════════════════════════════════ */
void Speed_PID_Init(void)   { Speed_PID_Reset(); }

void Speed_PID_Reset(void)
{
    integral_l   = 0.0f;
    integral_r   = 0.0f;
    last_error_l = 0.0f;
    last_error_r = 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Speed_PID_Update — 左右轮独立前馈+反馈速度控制
 * ═══════════════════════════════════════════════════════════════════════════ */
void Speed_PID_Update(float target_l_mm_s, float target_r_mm_s)
{
    /* ── 1. 读取编码器 (Q8.8 → mm/s) ── */
    int32_t raw_l = motor_read_encoder(MOTOR_L);
    int32_t raw_r = motor_read_encoder(MOTOR_R);
    actual_l_mm_s = (float)raw_l / 256.0f;
    actual_r_mm_s = (float)raw_r / 256.0f;

    /* ── 2. 左轮前馈 + PID ── */
    float ff_l  = target_l_mm_s * (4000.0f / Max_Speed_mm_s);
    float err_l = target_l_mm_s - actual_l_mm_s;

    /* 积分分离: 仅极端偏差时清零, 堵转时误差=Base_Speed(~150)不会被清 */
    if (err_l > 400.0f || err_l < -400.0f) {
        integral_l = 0.0f;
    } else {
        integral_l += err_l * SPEED_PID_DT;
        if (integral_l >  SPEED_INTEGRAL_MAX) integral_l =  SPEED_INTEGRAL_MAX;
        if (integral_l < -SPEED_INTEGRAL_MAX) integral_l = -SPEED_INTEGRAL_MAX;
    }
    float deriv_l = (err_l - last_error_l) / SPEED_PID_DT;
    float pid_l   = Speed_Kp * err_l + Speed_Ki * integral_l + Speed_Kd * deriv_l;
    last_error_l  = err_l;
    int32_t duty_l = (int32_t)(ff_l + pid_l);

    /* ── 3. 右轮前馈 + PID ── */
    float ff_r  = target_r_mm_s * (4000.0f / Max_Speed_mm_s);
    float err_r = target_r_mm_s - actual_r_mm_s;

    if (err_r > 400.0f || err_r < -400.0f) {
        integral_r = 0.0f;
    } else {
        integral_r += err_r * SPEED_PID_DT;
        if (integral_r >  SPEED_INTEGRAL_MAX) integral_r =  SPEED_INTEGRAL_MAX;
        if (integral_r < -SPEED_INTEGRAL_MAX) integral_r = -SPEED_INTEGRAL_MAX;
    }
    float deriv_r = (err_r - last_error_r) / SPEED_PID_DT;
    float pid_r   = Speed_Kp * err_r + Speed_Ki * integral_r + Speed_Kd * deriv_r;
    last_error_r  = err_r;
    int32_t duty_r = (int32_t)(ff_r + pid_r);

    /* ── 4. 限幅 ── */
    if (duty_l > 4000) duty_l = 4000;
    if (duty_l < 0)    duty_l = 0;
    if (duty_r > 4000) duty_r = 4000;
    if (duty_r < 0)    duty_r = 0;

    /* ── 5. 输出 ── */
    motor_set_direction(MOTOR_L, (duty_l > 0) ? MOTOR_FORWARD : MOTOR_STOP);
    motor_set_duty(MOTOR_L, (uint32_t)duty_l);
    motor_set_direction(MOTOR_R, (duty_r > 0) ? MOTOR_FORWARD : MOTOR_STOP);
    motor_set_duty(MOTOR_R, (uint32_t)duty_r);

    /* ── 6. 速度数据已保存在 actual_l/r_mm_s, 由主循环读取显示 ── */
}

float Speed_PID_GetActualL(void) { return actual_l_mm_s; }
float Speed_PID_GetActualR(void) { return actual_r_mm_s; }
