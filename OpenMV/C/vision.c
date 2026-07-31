#include "vision.h"

#include <stdbool.h>
#include <stdint.h>

#include "gimbal_motor.h"
#include "../MSPM0G3507/User/uart.h"

// ═══════════════════════════════════════════════════════════════
// 级联平衡控制器  —  Q8.8 定点数
//
//   外层 (位置环):  desired_tilt = KP·error + KD·vel   [°/px]
//   内层 (倾角环):  motor_speed  = KT·(desired − current) [°/s]
//
//   物理结构:  电机 →∫→ 臂角 → 管倾角 → 球加速度 →∫→ 球速 →∫→ 球位
//              └── 内层 ──┘ └──────── 外层 ────────────┘
//   每层只处理自己负责的积分器, 天然稳定, 独立可调。
// ═══════════════════════════════════════════════════════════════

// ── Q8.8 定点宏 ──
#define F8(f)      ((int16_t)((f) * 256.0f + 0.5f))
#define MUL8(a,b)  ((int16_t)(((int32_t)(a) * (int32_t)(b)) >> 8))
#define ABS8(x)    ((x) < 0 ? -(x) : (x))

// ── 周期 & 保护 ──
#define CTRL_PERIOD_MS       10
#define TIMEOUT_MS           200

// ═══════════════════════════════════════════════════════════════
// 外层 (位置环): position error → desired tilt
//   KP_POS: 1px 偏差 → 多少度倾角  (范围 0.1~0.5)
//   KD_POS: 1px/帧 球速 → 减多少度倾角 (阻尼, 范围 KP/4 ~ KP*2)
// ═══════════════════════════════════════════════════════════════
#define KP_POS               F8(2.0f)  // °/px
#define KD_POS               F8(18.0f)   // °/(px/frame)
#define MAX_TILT             F8(10.0f)  // 期望倾角上限
#define DEAD_ZONE            F8(0.5f)   // 位置死区 (偏差单位, =1像素)
#define VEL_EMA              F8(0.5f)   // 速度轻平滑 (α=0.5, 滞后≈1帧)

// ═══════════════════════════════════════════════════════════════
// 内层 (倾角环): tilt error → motor speed
//   K_TILT: 1° 倾角误差 → 多少 °/s 电机 (范围 5~20)
//   闭环时间常数 τ ≈ 1/K_TILT 秒
// ═══════════════════════════════════════════════════════════════
#define K_TILT               F8(10.0f)  // (°/s)/°
#define ARM_LIMIT            F8(10.0f)  // 臂角硬限位

// ── 方向 ──
#define DIRECTION_INVERT     1

extern volatile uint32_t sys_tick_ms;

// ── 状态 ──
static bool     frame_received = false;
static bool     first_frame    = true;
static uint32_t last_frame_ms  = 0;
static uint32_t last_ctrl_ms   = 0;
static int16_t  ball_pos       = 0;     // Q8.8 最近球位置 (int8→Q8.8)
static int16_t  ball_vel       = 0;     // Q8.8 球速估计 (轻 EMA)
static int16_t  desired_tilt   = 0;     // Q8.8 外层输出: 期望管倾角
static int16_t  current_tilt   = 0;     // Q8.8 臂角积分 ≈ 物理倾角


void process_deviation(void)
{
    // ═══════════════════════════════════════════════════════════
    // 1. 读 UART — 更新球位置和速度
    // ═══════════════════════════════════════════════════════════
    uint8_t frame_status;
    int8_t dev_x, dev_y;
    if (UART_get_deviations(&frame_status, &dev_x, &dev_y))
    {
        last_frame_ms  = sys_tick_ms;
        frame_received = true;

        uint8_t data_valid = frame_status & 0x01;

        if (data_valid)
        {
            int16_t new_pos = (int16_t)dev_x << 8;   // int8 → Q8.8

            if (first_frame)
            {
                ball_pos   = new_pos;
                ball_vel   = 0;
                first_frame = false;
            }
            else
            {
                // 原始帧间差分 → 轻 EMA 平滑 (α=0.5, 滞后≈1帧)
                int16_t raw_diff = new_pos - ball_pos;
                ball_vel += MUL8(VEL_EMA, raw_diff - ball_vel);
                ball_pos  = new_pos;
            }
        }
        else
        {
            // data_valid=0: 位置/速度向 0 衰减
            ball_pos -= ball_pos >> 2;
            ball_vel -= ball_vel >> 2;
        }
    }

    // ═══════════════════════════════════════════════════════════
    // 2. 超时 → 停机复位
    // ═══════════════════════════════════════════════════════════
    if (!frame_received || (sys_tick_ms - last_frame_ms) > TIMEOUT_MS)
    {
        gimbal_motor_stop(GIMBAL_MOTOR_R);
        ball_pos      = 0;
        ball_vel      = 0;
        desired_tilt  = 0;
        current_tilt  = 0;
        first_frame   = true;
        return;
    }

    // ═══════════════════════════════════════════════════════════
    // 3. 定周期 100Hz
    // ═══════════════════════════════════════════════════════════
    if (sys_tick_ms - last_ctrl_ms < CTRL_PERIOD_MS)
        return;
    last_ctrl_ms = sys_tick_ms;

    // ═══════════════════════════════════════════════════════════
    // 4. 外层 — 位置环: 球偏差 → 期望倾角
    //    desired_tilt = KP_POS·error + KD_POS·velocity
    //    死区内 desired_tilt=0, 内层会自然驱动臂回水平
    // ═══════════════════════════════════════════════════════════
    int16_t pos_error = ball_pos;                    // ref=0 即目标点
    int16_t abs_err   = ABS8(pos_error);

    if (abs_err < DEAD_ZONE)
    {
        desired_tilt = 0;
    }
    else
    {
        // 用 int32_t 直乘, 避免 MUL8 的 int16_t 截断溢出
        int32_t tilt = (((int32_t)KP_POS * (int32_t)pos_error) >> 8)
                     + (((int32_t)KD_POS * (int32_t)ball_vel) >> 8);

        // 限幅
        if (tilt > MAX_TILT)
            desired_tilt = MAX_TILT;
        else if (tilt < -MAX_TILT)
            desired_tilt = -MAX_TILT;
        else
            desired_tilt = (int16_t)tilt;
    }

    // ═══════════════════════════════════════════════════════════
    // 5. 内层 — 倾角环: 倾角误差 → 电机转速
    //    motor_speed = K_TILT·(desired_tilt − current_tilt)
    //    每次 10ms 都运行, 不依赖视觉帧率
    // ═══════════════════════════════════════════════════════════
    int16_t tilt_error = desired_tilt - current_tilt;

    // int32_t 直乘 — MUL8 在 K_TILT×tilt_error 时必溢出 int16_t
    int32_t motor_speed = ((int32_t)K_TILT * (int32_t)tilt_error) >> 8;

    int dir_sign = (motor_speed > 0) ? 1 : -1;
    int32_t abs_speed = (motor_speed > 0) ? motor_speed : -motor_speed;
    dir_sign *= DIRECTION_INVERT;

    // ═══════════════════════════════════════════════════════════
    // 6. 臂角积分 & 硬限位
    //    current_tilt += motor_speed × 0.01s (四舍五入)
    // ═══════════════════════════════════════════════════════════
    {
        int32_t deg_per_s = (abs_speed + 128) >> 8;     // Q8.8→int, 四舍五入
        current_tilt += (int16_t)(((deg_per_s * 256 + 50) / 100) * dir_sign);
    }

    if (current_tilt > ARM_LIMIT)
    {
        gimbal_motor_stop(GIMBAL_MOTOR_R);
        current_tilt = ARM_LIMIT;
        return;
    }
    if (current_tilt < -ARM_LIMIT)
    {
        gimbal_motor_stop(GIMBAL_MOTOR_R);
        current_tilt = -ARM_LIMIT;
        return;
    }

    // ═══════════════════════════════════════════════════════════
    // 7. 驱动电机 (<0.5°/s 停机, 避免无效微振)
    // ═══════════════════════════════════════════════════════════
    if (abs_speed < F8(0.5f))
    {
        gimbal_motor_stop(GIMBAL_MOTOR_R);
        return;
    }

    if (dir_sign > 0)
        gimbal_motor_set_dir(GIMBAL_MOTOR_R, GIMBAL_MOTOR_DIRECTION_FORWARD);
    else
        gimbal_motor_set_dir(GIMBAL_MOTOR_R, GIMBAL_MOTOR_DIRECTION_REVERSE);

    gimbal_motor_set_speed(GIMBAL_MOTOR_R,
        (uint8_t)((abs_speed > (255 << 8)) ? 255 : (abs_speed >> 8)));
    gimbal_motor_set_continuous(GIMBAL_MOTOR_R, 1);
    gimbal_motor_start(GIMBAL_MOTOR_R);
}


// ═══════════════════════════════════════════════════════════════
// 任务三: 定时开环序列 — 正转12° → -9° → 水平(3s) → -5°
//   不依赖视觉, 纯时间驱动倾角
// ═══════════════════════════════════════════════════════════════
#define T3_TILT_POS     F8(12.0f)     // +12°
#define T3_TILT_NEG     F8(-9.0f)     // -9°
#define T3_TILT_LEVEL   F8(0.0f)      // 水平
#define T3_TILT_NEG2    F8(-5.0f)     // -5°
#define T3_T1_MS        1500           // +12° → -9° 切换点
#define T3_T2_MS        1900           // -9° → 0° 切换点
#define T3_T3_MS        3140           // 水平 3s 后 → -5°

void process_deviation_task3(void)
{
    static uint32_t t3_start_ms  = 0;
    static uint32_t t3_last_ctrl = 0;
    static int16_t  t3_desired   = 0;       // Q8.8 期望倾角
    static int16_t  t3_current   = 0;       // Q8.8 臂角积分
    static uint8_t  t3_state     = 0;       // 0=init,1=+10°,2=-20°,3=level

    // ── 1. 状态机: 纯时间驱动 ──
    if (t3_state == 0)
    {
        t3_start_ms = sys_tick_ms;
        t3_state    = 1;
    }

    uint32_t elapsed = sys_tick_ms - t3_start_ms;

    if (elapsed < T3_T1_MS)
        t3_desired = T3_TILT_POS;           // 0~1.5s: +12°
    else if (elapsed < T3_T2_MS)
        t3_desired = T3_TILT_NEG;           // 1.5~1.9s: -9°
    else if (elapsed < T3_T3_MS)
        t3_desired = T3_TILT_LEVEL;         // 1.9~4.9s: 水平 (3秒)
    else
        t3_desired = T3_TILT_NEG2;          // 4.9s+: -2°

    // ── 2. 超时保护 (OpenMV 断联时也继续工作) ──
    {
        uint8_t dummy_status; int8_t dummy_x, dummy_y;
        static uint32_t last_frame = 0;
        static bool got_frame = false;
        if (UART_get_deviations(&dummy_status, &dummy_x, &dummy_y))
            { last_frame = sys_tick_ms; got_frame = true; }
        if (got_frame && (sys_tick_ms - last_frame) > TIMEOUT_MS)
        {
            gimbal_motor_stop(GIMBAL_MOTOR_R);
            t3_current = 0;
            t3_state   = 0;
            return;
        }
    }

    // ── 3. 100Hz ──
    if (sys_tick_ms - t3_last_ctrl < CTRL_PERIOD_MS)
        return;
    t3_last_ctrl = sys_tick_ms;

    // ── 4. 内层: 倾角跟踪 ──
    int16_t tilt_error = t3_desired - t3_current;
    int32_t motor_speed = ((int32_t)K_TILT * (int32_t)tilt_error) >> 8;

    int dir_sign = (motor_speed > 0) ? 1 : -1;
    int32_t abs_speed = (motor_speed > 0) ? motor_speed : -motor_speed;
    dir_sign *= DIRECTION_INVERT;

    // ── 5. 臂角积分 ──
    {
        int32_t deg_per_s = (abs_speed + 128) >> 8;
        t3_current += (int16_t)(((deg_per_s * 256 + 50) / 100) * dir_sign);
    }
    if (t3_current > ARM_LIMIT)
    { gimbal_motor_stop(GIMBAL_MOTOR_R); t3_current = ARM_LIMIT; return; }
    if (t3_current < -ARM_LIMIT)
    { gimbal_motor_stop(GIMBAL_MOTOR_R); t3_current = -ARM_LIMIT; return; }

    // ── 6. 驱动 ──
    if (abs_speed < F8(0.5f)) { gimbal_motor_stop(GIMBAL_MOTOR_R); return; }
    if (dir_sign > 0)
        gimbal_motor_set_dir(GIMBAL_MOTOR_R, GIMBAL_MOTOR_DIRECTION_FORWARD);
    else
        gimbal_motor_set_dir(GIMBAL_MOTOR_R, GIMBAL_MOTOR_DIRECTION_REVERSE);
    gimbal_motor_set_speed(GIMBAL_MOTOR_R,
        (uint8_t)((abs_speed > (255 << 8)) ? 255 : (abs_speed >> 8)));
    gimbal_motor_set_continuous(GIMBAL_MOTOR_R, 1);
    gimbal_motor_start(GIMBAL_MOTOR_R);
}
