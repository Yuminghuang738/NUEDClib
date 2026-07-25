#ifndef ANGLE_CONTROL_H
#define ANGLE_CONTROL_H

#include <stdint.h>

/* ============================================================
 * 角度闭环控制模块 — 基于 JY61P Yaw 角的位置式 PID
 * 调用周期: 10ms (100Hz), 在定时中断或主循环中调用
 * 符号约定: correction > 0 → 向右纠偏 (左轮加速, 右轮减速)
 *           correction < 0 → 向左纠偏 (右轮加速, 左轮减速)
 * ============================================================ */

/* ── PID 运算周期 (秒), 硬编码 dt 避免浮点重复计算 ── */
#define ANGLE_PID_DT         0.01f

/* ── PID 参数 (可根据实车调试在线修改) ── */
extern float Angle_Kp;
extern float Angle_Ki;
extern float Angle_Kd;

/* ── 积分分离阈值: 角度误差绝对值 > 此值时强制 Ki=0 ── */
#define ANGLE_SEPARATE_DEG   60.0f

/* ── 积分限幅 (±) ── */
#define ANGLE_INTEGRAL_MAX   100.0f

/* ── 角度死区: 误差绝对值 < 此值时直接清零, 避免轻微抖动 ── */
#define ANGLE_DEAD_ZONE      1.5f

/* ── PID 输出差速修正量限幅 (±) ── */
#define ANGLE_OUTPUT_MAX     2000.0f

/* ── 最大最终速度 (防止溢出) ── */
#define ANGLE_SPEED_MAX      4000

/* ── 直线行驶增益缩放: base_speed>0 时 correction 乘此系数 ── */
#define STRAIGHT_GAIN_SCALE  0.25f

/* ── API ── */

/*
 * 角度闭环主调用接口
 * target_angle : 目标航向角 (°), 范围 0~360
 * base_speed   : 基础直线车速 (PWM 占空比值, 0~4000)
 *
 * 调用方需在 10ms 定时中断/主循环中固定周期调用。
 */
void Angle_Control_Update(float target_angle, int16_t base_speed);

/*
 * 重置 PID 历史状态 (积分累加、上次误差)
 * 用于模式切换、停车后重新起步等场景
 */
void Angle_PID_Reset(void);

/*
 * 由外部提供的角度数据源 (JY61P 驱动实现)
 * 返回当前 Yaw 偏航角, 单位: °
 */
extern float JY61P_Get_Yaw(void);

/*
 * 由外部提供的电机速度输出接口 (电机驱动实现)
 * left  : 左轮 PWM 占空比 (0~4000)
 * right : 右轮 PWM 占空比 (0~4000)
 *
 * 【提醒】请在电机驱动模块中自行实现此函数。
 */
extern void Motor_SetSpeed(int16_t left, int16_t right);

#endif
