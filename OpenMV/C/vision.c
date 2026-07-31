#include "vision.h"

#include <stdbool.h>
#include <stdint.h>

#include "gimbal_motor.h"
#include "../MSPM0G3507/User/uart.h"

// ═══════════════════════════════════════════════════════════════
// 视觉伺服 — 速度式 PD 控制器 + EMA 滤波
//
// PD 公式:
//   effective = Kp·error + Kd·raw_deriv
//   speed     = |effective|  (°/s)
//   direction = sign(effective)
//
// P 项: error = EMA(raw_dev)  ← 平滑位置, 电机不抖
// D 项: deriv = raw_dev - last_raw_dev  ← 原始速度, 零滞后
//   球往中心冲(deriv<0) → effective 减小/变号 → 提前反向刹车
//   球往外跑(deriv>0)   → effective 增大      → 加速响应
//
// continuous 模式持续运行, 方向随 effective 符号瞬间翻转
// ═══════════════════════════════════════════════════════════════

#define CTRL_PERIOD_MS   10      // 100Hz
#define DEAD_ZONE        3.0f    // ±3px 死区
#define TIMEOUT_MS       200
#define ARM_ANGLE_MAX    30.0f
#define ARM_ANGLE_MIN   -30.0f

// ── EMA: 只用于 P 项位置平滑, D 项走原始值不受影响 ──
#define EMA_ALPHA  0.7f

// ── PD 参数 ──
// effective = Kp·EMA(error) + Kd·raw_deriv
// KP: 偏差→速度的灵敏度. 太小=球滚不起来, 太大=冲过头
// KD: 球速度的阻尼.    太小=来回摆,       太大=反应迟钝
//     注意: raw_deriv 单位是 px/frame(~20ms), 比 EMA deriv 大约 2~3 倍
#define KP_SPEED   0.5f
#define KD         200.0f

// ── 速度平滑 (可选): 1.0=不过滤, 0.5=轻度平滑防毛刺 ──
#define VEL_ALPHA  1.0f

#define SPEED_MAX   25.0f   // 最大转速 °/s
#define SPEED_MIN    5.0f   // 最低转速

// ── 方向调试: 1=正常, -1=反转 ──
#define DIRECTION_INVERT  1

extern volatile uint32_t sys_tick_ms;

static bool     frame_received  = false;
static uint32_t last_frame_ms   = 0;
static uint32_t last_ctrl_ms    = 0;
static float    ema_dev_x       = 0.0f;
static float    last_raw_dev_x  = 0.0f;   // D 项用: 上一帧原始偏差
static float    cached_raw_deriv = 0.0f;  // D 项用: 缓存的原始速度
static float    arm_angle       = 0.0f;   // 软件跟踪物理角度


void process_deviation(void)
{
    // ── 1. 读 UART → EMA(P项用) + 原始速度(D项用) ──
    uint8_t frame_status;
    int8_t dev_x, dev_y;
    if (UART_get_deviations(&frame_status, &dev_x, &dev_y))
    {
        last_frame_ms  = sys_tick_ms;
        frame_received = true;

        // P 项: EMA 平滑位置
        ema_dev_x = EMA_ALPHA * (float)dev_x
                  + (1.0f - EMA_ALPHA) * ema_dev_x;

        // D 项: 原始速度 (无 EMA 滞后, 可选 VEL_ALPHA 轻微平滑防毛刺)
        float raw_deriv = (float)dev_x - last_raw_dev_x;
        last_raw_dev_x = (float)dev_x;
        cached_raw_deriv = VEL_ALPHA * raw_deriv
                         + (1.0f - VEL_ALPHA) * cached_raw_deriv;
    }

    // ── 2. 超时 → 停机复位 ──
    if (!frame_received || (sys_tick_ms - last_frame_ms) > TIMEOUT_MS)
    {
        gimbal_motor_stop(GIMBAL_MOTOR_R);
        ema_dev_x        = 0.0f;
        last_raw_dev_x   = 0.0f;
        cached_raw_deriv = 0.0f;
        arm_angle        = 0.0f;
        return;
    }

    // ── 3. 定周期 100Hz ──
    if (sys_tick_ms - last_ctrl_ms < CTRL_PERIOD_MS)
        return;
    last_ctrl_ms = sys_tick_ms;

    // ── 4. PD 控制 ──
    float error   = ema_dev_x;                   // P: 平滑位置
    float abs_err = (error > 0.0f) ? error : -error;

    // 死区 → 停电机, D 项清零防 kick
    if (abs_err < DEAD_ZONE)
    {
        gimbal_motor_stop(GIMBAL_MOTOR_R);
        cached_raw_deriv = 0.0f;
        return;
    }

    // effective = Kp·EMA(error) + Kd·raw_deriv
    // raw_deriv: 球在回中(负) → effective 减小/变号 → 提前反向刹车
    //            球在远离(正) → effective 增大      → 加速响应
    float effective = KP_SPEED * error + KD * cached_raw_deriv;

    // speed = |effective|, direction = sign(effective) × 反转
    float speed = (effective > 0.0f) ? effective : -effective;
    if (speed > SPEED_MAX) speed = SPEED_MAX;
    if (speed < SPEED_MIN)
    {
        gimbal_motor_stop(GIMBAL_MOTOR_R);
        return;
    }

    int dir_sign = (effective > 0.0f) ? 1 : -1;
    dir_sign *= DIRECTION_INVERT;

    // ── 5. 跟踪物理角度, 硬限幅 ±30° ──
    float dt = CTRL_PERIOD_MS / 1000.0f;   // 0.01s
    arm_angle += speed * dt * (float)dir_sign;

    if (arm_angle > ARM_ANGLE_MAX)
    {
        gimbal_motor_stop(GIMBAL_MOTOR_R);
        arm_angle = ARM_ANGLE_MAX;
        return;
    }
    if (arm_angle < ARM_ANGLE_MIN)
    {
        gimbal_motor_stop(GIMBAL_MOTOR_R);
        arm_angle = ARM_ANGLE_MIN;
        return;
    }

    // ── 6. 驱动电机 continuous 模式 ──
    if (dir_sign > 0)
        gimbal_motor_set_dir(GIMBAL_MOTOR_R, GIMBAL_MOTOR_DIRECTION_FORWARD);
    else
        gimbal_motor_set_dir(GIMBAL_MOTOR_R, GIMBAL_MOTOR_DIRECTION_REVERSE);

    gimbal_motor_set_speed(GIMBAL_MOTOR_R, (uint8_t)speed);
    gimbal_motor_set_continuous(GIMBAL_MOTOR_R, 1);
    gimbal_motor_start(GIMBAL_MOTOR_R);
}
