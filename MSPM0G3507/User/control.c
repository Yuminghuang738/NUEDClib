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
extern volatile uint32_t encoder_total;
extern volatile int status;
extern volatile uint8_t nostop_mode;

volatile uint8_t  cross_cnt         = 0;
volatile uint8_t  reverse_brake_cnt  = 0;
volatile uint8_t  stop_armed         = 0;   /* 3s 后置 1 */

/* 转向方向翻转开关: 如果左传感器见黑线时车向右转, 改为 -1 */
#define TRACE_POLARITY   -1

/* 机械偏航补偿: 车向右偏 → 正值(补右轮); 车向左偏 → 负值(补左轮) */
//#define RIGHT_BIAS_MM_S   25.0f

/* ═══════════════════════════════════════════════════════════════════════════
 *  外环位置 PD 参数 (extern — 在线可调)
 *
 *  turn = Trace_Kp * error + Trace_Kd * (error - last_error)
 *  error: trace_get_error() 返回值, 约 -10..+10
 *  turn:  速度修正量 mm/s
 * ═══════════════════════════════════════════════════════════════════════════ */
float Trace_Kp = 35.0f;
float Trace_Kd = 15.0f;

/* 不停车循迹参数 (KEY_4: 任务4/5/6, 带平衡球, 减小速度+平顺PID 避免球摇摆) */
#define NOSTOP_TRACE_KP     15.0f
#define NOSTOP_TRACE_KD      6.0f
#define NOSTOP_BASE_SPEED  200.0f

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
    /* 停车状态: 反向脉冲 → 制动 */
    if (status == 0) {
        Speed_PID_Reset();
        if (reverse_brake_cnt > 0) {
            reverse_brake_cnt--;
            motor_set_direction(MOTOR_L, MOTOR_BACKWARD);
            motor_set_direction(MOTOR_R, MOTOR_BACKWARD);
            motor_set_duty(MOTOR_L, 2000);
            motor_set_duty(MOTOR_R, 2000);
        } else {
            motor_set_direction(MOTOR_L, MOTOR_STOP);
            motor_set_direction(MOTOR_R, MOTOR_STOP);
            motor_set_duty(MOTOR_L, 4000);
            motor_set_duty(MOTOR_R, 4000);
        }
        first_pos_call     = 1;
        last_valid_error   = 0;
        return;
    }

    trace_read();

    /* X7 停止线检测 — ISR 内同步 + 里程门禁 + 不停车模式跳过 */
    if (!nostop_mode && status == 1 && trace_data[5] == 1 && encoder_total > 10000) {
        status = 0;
        reverse_brake_cnt = 0;
    }

    int8_t  error  = trace_get_error();
    uint8_t active = trace_get_active();

    if (active == 0) {
        error = last_valid_error;
        first_pos_call = 1;
    }

    if (active >= 5) {
        error = last_valid_error;
    } else {
        last_valid_error = error;
    }

    error = (int8_t)((int16_t)error * TRACE_POLARITY);

    float kp   = nostop_mode ? NOSTOP_TRACE_KP   : Trace_Kp;
    float kd   = nostop_mode ? NOSTOP_TRACE_KD   : Trace_Kd;
    float base = nostop_mode ? NOSTOP_BASE_SPEED : Base_Speed_mm_s;

    float turn;
    if (first_pos_call) {
        turn = kp * (float)error;
        first_pos_call = 0;
    } else {
        turn = kp * (float)error
             + kd * (float)(error - last_pos_error);
    }
    last_pos_error = error;

    float target_l = base - turn;
    float target_r = base + turn;
    if (target_l < 30.0f) target_l = 30.0f;
    if (target_r < 30.0f) target_r = 30.0f;

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
