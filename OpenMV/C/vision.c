#include "vision.h"

#include <stdbool.h>
#include <stdint.h>

#include "gimbal_motor.h"
#include "../MSPM0G3507/User/uart.h"

// ═══════════════════════════════════════════════════════════════
// 视觉伺服 PD 控制器 + EMA 滤波
//
// EMA (指数移动平均): 低通滤波, 抑制 OpenMV 检测噪声
//   ema = α·new + (1-α)·old
//   α 大 → 响应快但抖; α 小 → 平滑但迟钝
//
// PD (比例-微分):
//   desired = Kp·error + Kd·d(error)/dt
//   P → 像素偏差线性映射为目标倾角 (位置式, 不会累积)
//   D → 误差变化率作为阻尼 (球正回中时刹车, 防过冲振荡)
//
// 控制流程:
//   100Hz 定周期 → 读 EMA 偏差 → PD 算目标角度 (±30°限幅)
//   → 增量式趋近目标 → 驱动云台电机 R
// ═══════════════════════════════════════════════════════════════

// ── 控制周期 & 保护 ──
#define CTRL_PERIOD_MS   10      // 控制周期 100Hz
#define DEAD_ZONE        3.0f    // 死区 ±3px, 球在死区内不动作
#define TIMEOUT_MS       200     // 200ms 无数据 → 停机复位
#define ARM_ANGLE_MAX    30.0f   // 硬限幅上界
#define ARM_ANGLE_MIN   -30.0f   // 硬限幅下界

// ── EMA 参数 ──
#define EMA_ALPHA  0.3f          // 平滑系数, 约 3~5 帧收敛

// ── PD 参数 (需实车调试) ──
#define KP  0.15f                // 比例: 1px 偏差 → 0.15° 倾角
#define KD  0.35f                // 微分: 误差变化率阻尼

// ── 方向调试开关: 1=正常, -1=反转 ──
#define DIRECTION_INVERT  1

extern volatile uint32_t sys_tick_ms;

static bool     frame_received = false;
static uint32_t last_frame_ms  = 0;
static uint32_t last_ctrl_ms   = 0;
static float    ema_dev_x      = 0.0f;
static float    last_error     = 0.0f;
static float    arm_angle      = 0.0f;   // 软件跟踪的当前电机角度


void process_deviation(void)
{
    // ── 第 1 步: 读 UART → 更新 EMA 滤波值 ──
    uint8_t frame_status;
    int8_t dev_x, dev_y;
    if (UART_get_deviations(&frame_status, &dev_x, &dev_y))
    {
        last_frame_ms  = sys_tick_ms;
        frame_received = true;
        ema_dev_x = EMA_ALPHA * (float)dev_x
                  + (1.0f - EMA_ALPHA) * ema_dev_x;
    }

    // ── 第 2 步: 超时保护 → 停机并复位全部状态 ──
    if (!frame_received || (sys_tick_ms - last_frame_ms) > TIMEOUT_MS)
    {
        gimbal_motor_stop(GIMBAL_MOTOR_R);
        ema_dev_x   = 0.0f;
        last_error  = 0.0f;
        arm_angle   = 0.0f;
        return;
    }

    // ── 第 3 步: 定周期执行 100Hz ──
    if (sys_tick_ms - last_ctrl_ms < CTRL_PERIOD_MS)
        return;
    last_ctrl_ms = sys_tick_ms;

    // ── 第 4 步: PD 计算目标角度 ──
    float error, desired;

    if (ema_dev_x > -DEAD_ZONE && ema_dev_x < DEAD_ZONE)
    {
        // 球在死区内 → 目标归零, D 项清零防止进入死区时 kick
        error   = 0.0f;
        desired = 0.0f;
        last_error = 0.0f;
    }
    else
    {
        error   = ema_dev_x;
        // D 项: 误差变化率 (px/10ms), 为正=球在远离中心, 为负=球在回中
        desired = KP * error + KD * (error - last_error);

        // 硬限幅 ±30°
        if (desired > ARM_ANGLE_MAX)  desired = ARM_ANGLE_MAX;
        if (desired < ARM_ANGLE_MIN)  desired = ARM_ANGLE_MIN;

        last_error = error;
    }

    // 方向反转
    desired *= (float)DIRECTION_INVERT;

    // ── 第 5 步: 增量式趋近目标角度 ──
    float delta = desired - arm_angle;

    // <0.1° 忽略 (避免 uint8_t 截断导致的无效振荡)
    if (delta > -0.1f && delta < 0.1f)
        return;

    // 单次步进上限 14° (gimbal_motor_set_angle 参数为 uint8_t, 最多 14.3°)
    if (delta >  14.0f) delta =  14.0f;
    if (delta < -14.0f) delta = -14.0f;

    if (delta > 0.0f)
    {
        gimbal_motor_set_dir(GIMBAL_MOTOR_R, GIMBAL_MOTOR_DIRECTION_FORWARD);
        uint8_t angle_cmd = (uint8_t)delta;          // 截断 <1° 的零头 (可接受的粒度)
        gimbal_motor_set_speed(GIMBAL_MOTOR_R, 10);
        gimbal_motor_set_angle(GIMBAL_MOTOR_R, angle_cmd);
        arm_angle += (float)angle_cmd;
    }
    else
    {
        gimbal_motor_set_dir(GIMBAL_MOTOR_R, GIMBAL_MOTOR_DIRECTION_REVERSE);
        uint8_t angle_cmd = (uint8_t)(-delta);
        gimbal_motor_set_speed(GIMBAL_MOTOR_R, 10);
        gimbal_motor_set_angle(GIMBAL_MOTOR_R, angle_cmd);
        arm_angle -= (float)angle_cmd;
    }
}
