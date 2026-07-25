#include "angle_control.h"
#include "jy61p_port.h"

extern volatile uint32_t sys_tick_ms;

/* ============================================================
 * 角度闭环控制 — 位置式 PID 实现
 *
 * 角度来源:   JY61P 姿态传感器 UART 串口输出
 * 控制对象:   两轮差速小车航向角 (Yaw)
 * 执行机构:   TB6612 + 直流减速电机 (左右轮差速)
 * PID 周期:   10ms (100Hz), 由外部定时中断保证
 * ============================================================ */

/* ── PID 参数默认值 (可在运行时通过 extern 在线调整) ── */
float Angle_Kp = 40.0f;   //精度最好的是40 5 3
float Angle_Ki = 5.0f;
float Angle_Kd = 3.0f;

/* ── PID 内部状态 (静态变量, 外部不可直接访问) ── */
static float integral      = 0.0f;   /* 积分累加量 */
static float last_error    = 0.0f;   /* 上一次角度误差 */
static float last_angle    = 0.0f;   /* 上一次实测角度 (用于 D 项微分先行) */
static uint8_t first_call  = 1;      /* 首次调用标志, 用于初始化 last_angle */
static uint32_t last_call_ms = 0;    /* 上次调用时刻, 用于计算实际 dt */

/* ============================================================
 * Angle_PID_Reset — 清空积分和历史误差
 * 适用场景: 循迹/角度模式切换时、停车重新起步时
 * ============================================================ */
void Angle_PID_Reset(void)
{
    integral     = 0.0f;
    last_error   = 0.0f;
    last_angle   = 0.0f;
    first_call   = 1;
    last_call_ms = 0;
}

/* ============================================================
 * normalize_error — 计算最短角度误差 (处理 ±180° 跳变)
 *
 * JY61P Yaw 角输出范围: -180° ~ +180°
 * 直接相减可能产生 >180° 的误差, 导致 PID 震荡发散
 * 本函数将误差强制映射到 [-180°, +180°] 区间
 * ============================================================ */
static float normalize_error(float target, float current)
{
    float error = target - current;

    /* 循环消去 ±180° 跳变, 找到最短旋转路径 */
    while (error >  180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    return error;
}

/* ============================================================
 * Angle_Control_Update — 角度闭环 PID 主函数
 *
 * 调用周期: 10ms (由外部定时中断/主循环保证)
 *
 * 控制流程:
 *   1. 保护检查 (base_speed == 0 → 退出)
 *   2. 读取当前 Yaw 角 → 计算最短角度误差
 *   3. 死区判定 (|error| < 0.5° → 清零)
 *   4. 位置式 PID 计算 (P + I + D)
 *       - 积分分离: |error| > 30° → Ki = 0
 *       - 积分限幅: integral ∈ [-100, +100]
 *   5. 输出限幅: correction ∈ [-OUTPUT_MAX, +OUTPUT_MAX]
 *   6. 差速合成: left = base + corr, right = base - corr
 *   7. 输出限速后调用 Motor_SetSpeed
 * ============================================================ */
void Angle_Control_Update(float target_angle, int16_t base_speed)
{
    float current_angle;
    float error;
    float p_out, i_out, d_out;
    float correction;
    float dt;
    int16_t left_speed, right_speed;

    /* ── 1. 计算实际 dt (秒), 消除 OLED 刷新导致的周期抖动 ── */
    {
        uint32_t now = sys_tick_ms;
        uint32_t delta_ms;
        if (last_call_ms == 0) {
            dt = ANGLE_PID_DT;          /* 首次调用使用默认周期 */
        } else {
            delta_ms = now - last_call_ms;
            dt = (float)delta_ms / 1000.0f;
            if (dt > 0.1f) dt = ANGLE_PID_DT;  /* 超长间隔限幅, 防 D 项巨浪 */
        }
        last_call_ms = now;
    }

    /* ── 2. 获取当前 Yaw 角 ── */
    current_angle = JY61P_Get_Yaw();

    /* 计算最短角度误差 (处理 ±180° 跳变) */
    error = normalize_error(target_angle, current_angle);

    /* ── 3. 角度死区: 误差极小直接置零, 避免微小抖动引发 PID 输出 ── */
    if (error < ANGLE_DEAD_ZONE && error > -ANGLE_DEAD_ZONE) {
        error = 0.0f;
    }

    /* ── 4. 位置式 PID 运算 ── */

    /* 4a. 比例项 */
    p_out = Angle_Kp * error;

    /* 4b. 积分项 — 积分分离 + 过零清零 */
    if (error > ANGLE_SEPARATE_DEG || error < -ANGLE_SEPARATE_DEG) {
        i_out = 0.0f;
    } else {
        /* 误差符号翻转 = 冲过目标, 清积分防止反向猛推 */
        if ((error > 0.0f && last_error < 0.0f) ||
            (error < 0.0f && last_error > 0.0f)) {
            integral = 0.0f;
        }

        integral += Angle_Ki * error * dt;

        if (integral >  ANGLE_INTEGRAL_MAX) integral =  ANGLE_INTEGRAL_MAX;
        if (integral < -ANGLE_INTEGRAL_MAX) integral = -ANGLE_INTEGRAL_MAX;

        i_out = integral;
    }

    /* 4c. 微分项 — 微分先行 (基于测量值微分, 避免设定值突变冲击) */
    if (first_call) {
        last_angle = current_angle;
        first_call = 0;
    }
    d_out = Angle_Kd * (last_angle - current_angle) / dt;  /* 使用实际 dt */
    last_angle = current_angle;

    /* ── 5. PID 输出合成 ── */
    correction = p_out + i_out + d_out;

    /* 直线行驶时等比缩小增益: 前进中差速转向效率远高于原地,
       同样的 correction 在直行时会导致 S 形摆动, 需缩至 ~1/4 */
    if (base_speed != 0) {
        correction *= STRAIGHT_GAIN_SCALE;
    }

    /* ── 6. 输出限幅: 限制最大差速纠偏力度 ── */
    if (correction >  ANGLE_OUTPUT_MAX) correction =  ANGLE_OUTPUT_MAX;
    if (correction < -ANGLE_OUTPUT_MAX) correction = -ANGLE_OUTPUT_MAX;

    /* ── 7. 差速合成 ──
     * base_speed=0 时: left=-corr, right=+corr → 原地差速旋转
     * correction > 0: 左轮反转/右轮正转 → 车体逆时针旋转 (向左)
     * correction < 0: 左轮正转/右轮反转 → 车体顺时针旋转 (向右)
     */
    left_speed  = (int16_t)((float)base_speed - correction);
    right_speed = (int16_t)((float)base_speed + correction);

    /* ── 8. 最终速度限幅: 防止 PWM 占空比溢出 ── */
    if (left_speed  >  ANGLE_SPEED_MAX) left_speed  =  ANGLE_SPEED_MAX;
    if (left_speed  < -ANGLE_SPEED_MAX) left_speed  = -ANGLE_SPEED_MAX;
    if (right_speed >  ANGLE_SPEED_MAX) right_speed =  ANGLE_SPEED_MAX;
    if (right_speed < -ANGLE_SPEED_MAX) right_speed = -ANGLE_SPEED_MAX;

    /* ── 9. 保存本次误差 (供下次 D 项或调试使用) ── */
    last_error = error;

    /* ── 10. 输出到电机驱动层 ── */
    Motor_SetSpeed(left_speed, right_speed);
}
