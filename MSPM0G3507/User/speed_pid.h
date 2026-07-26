#ifndef SPEED_PID_H
#define SPEED_PID_H

#include <stdint.h>

/* ── 外环位置 PD 参数 (extern, 运行时在线可调) ── */
extern float Trace_Kp;
extern float Trace_Kd;

/* ── 内环速度 PID 参数 (extern, 运行时在线可调) ── */
extern float Speed_Kp;
extern float Speed_Ki;
extern float Speed_Kd;

/* ── 基础目标线速度 (mm/s) ── */
extern float Base_Speed_mm_s;

/* ── 电机最高线速度 (mm/s, 占空比=4000 时的实际速度) ── */
extern float Max_Speed_mm_s;

void Speed_PID_Init(void);
void Speed_PID_Reset(void);

/*
 * 速度 PID 更新 — 每个控制周期调用一次 (20ms)
 * target_l_mm_s : 左轮目标线速度 (mm/s)
 * target_r_mm_s : 右轮目标线速度 (mm/s)
 *
 * 内部: 编码器反馈 → 前馈 + PID → motor_set_duty/motor_set_direction
 */
void Speed_PID_Update(float target_l_mm_s, float target_r_mm_s);

float Speed_PID_GetActualL(void);
float Speed_PID_GetActualR(void);

#endif
