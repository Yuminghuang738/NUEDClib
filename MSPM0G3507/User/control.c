#include "control.h"
#include "control_config.h"
#include "trace.h"
#include "motor.h"
#include "speed_pid.h"

#ifdef CONTROL_OPEN_LOOP
#include "control_open.h"
#else
#include "control_closed.h"
#endif

extern volatile uint32_t sys_tick_ms;
extern volatile int status;

/* 转向方向翻转开关: 如果左传感器见黑线时车向右转, 改为 -1 */
#define TRACE_POLARITY    1

/* 机械偏航补偿: 车向右偏 → 正值(补右轮); 车向左偏 → 负值(补左轮) */
//#define RIGHT_BIAS_MM_S   25.0f

/* ═══════════════════════════════════════════════════════════════════════════
 *  外环位置 PD 参数 (extern — 在线可调)
 *
 *  turn = Trace_Kp * error + Trace_Kd * (error - last_error)
 *  error: trace_get_error() 返回值, 约 -10..+10
 *  turn:  速度修正量 mm/s
 * ═══════════════════════════════════════════════════════════════════════════ */
float Trace_Kp = 29.0f;
float Trace_Kd = 3.0f;

static int8_t  last_pos_error  = 0;
static int8_t  last_valid_error = 0;   /* 丢线时保持最后有效误差方向 */
static uint8_t first_pos_call  = 1;

void control_init(void)
{
    control_variant_init();
    Speed_PID_Init();
}

void control_update(void)
{
    if (angle_ctrl_active) {
        if (sys_tick_ms - last_angle_ctrl_ms > 100) {
            angle_ctrl_active = 0;
        } else {
            return;
        }
    }

    /* 停车状态: 刹车 + 返回 */
    if (status == 0) {
        Speed_PID_Reset();
        motor_set_direction(MOTOR_L, MOTOR_STOP);
        motor_set_direction(MOTOR_R, MOTOR_STOP);
        motor_set_duty(MOTOR_L, 0);
        motor_set_duty(MOTOR_R, 0);
        first_pos_call     = 1;
        last_valid_error   = 0;
        return;
    }

    /* 角度闭环由主循环 Angle_Control_Update 接管, ISR 不干预 */
    if (status == 2) return;

    trace_read();
    int8_t  error  = trace_get_error();
    uint8_t active = trace_get_active();

    if (active == 0) {
        Speed_PID_Reset();
        motor_set_direction(MOTOR_L, MOTOR_STOP);
        motor_set_direction(MOTOR_R, MOTOR_STOP);
        motor_set_duty(MOTOR_L, 0);
        motor_set_duty(MOTOR_R, 0);
        control_variant_reset();
        first_pos_call   = 1;
        last_valid_error = 0;
        return;
    }

    if (active == 4) {
        error = last_valid_error;
    } else {
        last_valid_error = error;
    }

    error = (int8_t)((int16_t)error * TRACE_POLARITY);

    float turn;
    if (first_pos_call) {
        turn = Trace_Kp * (float)error;
        first_pos_call = 0;
    } else {
        turn = Trace_Kp * (float)error
             + Trace_Kd * (float)(error - last_pos_error);
    }
    last_pos_error = error;

    float target_l = Base_Speed_mm_s - turn;
    float target_r = Base_Speed_mm_s + turn;
    if (target_l < 30.0f) target_l = 100.0f;
    if (target_r < 30.0f) target_r = 100.0f;

    Speed_PID_Update(target_l, target_r);
}

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
