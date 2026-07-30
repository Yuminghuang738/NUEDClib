#include "vision.h"

#include <stdbool.h>
#include <stdint.h>

#include "gimbal_motor.h"
#include "../MSPM0G3507/User/uart.h"

// ═══════════════════════════════════════════════════════════════
// 平衡滚球 — PD 控制器 + 步进电机定位
// ═══════════════════════════════════════════════════════════════
//
// 机械结构:
//   步进电机 → 绕线盘 → 绳子 → 摆杆顶端
//   X 轴偏差 → 绳长调节 (收绳/放绳) → 摆杆倾角变化 → 钢球回正
//
// 控制算法: PD (比例-微分)
//   output = KP * error + KD * velocity
//   error   = ball_cx - ref_x  (位置偏差, 像素)
//   velocity = error - last_error  (速度估计, 像素/帧)
//
//   P 项: 位置偏离 → 纠正方向
//   D 项: 球冲太快 → 加大阻尼;  球在回正 → 减小力度防过冲
//   → 这就是"反复拉松"的阻尼平衡效果
//
// 方向约定 (需根据实际绕线方向确认):
//   deviation > 0 (球偏右) → FORWARD  (放绳, 右侧下降)
//   deviation < 0 (球偏左) → REVERSE (收绳, 右侧上升)
//   如果方向反了, 交换 gimbal_motor_set_dir 的 FORWARD/REVERSE 即可
//
// 调参顺序:
//   1. KP_X: 从 1 开始, 逐步增大直到钢球能回正
//   2. KD_X: 从 0.5 开始, 增大直到不再来回震荡
//   3. K_ANGLE_X: 调整每像素偏差对应的电机角度 (与机械臂长度/绕线盘半径相关)
//   4. DEAD_ZONE: 平衡容许误差 (越小越精确但越容易震荡)

// ── PD 控制器增益 ──
#define VISION_KP_X               2       // 比例增益 (P)
#define VISION_KD_X               2       // 微分增益 (D) — 阻尼强度

// ── 步进定位参数 ──
#define VISION_K_ANGLE_X          2.0f    // deg/pixel: 每像素偏差对应电机角度
#define VISION_K_ANGLE_Q8         512     // = K_ANGLE_X * 256, Q8 定点
#define VISION_STEP_SPEED_X       60      // 固定步进速度 (deg/s)
#define VISION_MAX_ANGLE_PER_FRAME 12     // 单帧最大角度 (度), 防止过冲

// ── 死区 ──
#define VISION_DEAD_ZONE_ENTER_X  2       // 进入死区阈值 (像素)
#define VISION_DEAD_ZONE_EXIT_X   4       // 退出死区阈值 (像素)

// ── 共用超时与滤波 ──
#define VISION_TIMEOUT_MS       200       // 失联超时 (ms)
#define VISION_EMA_ALPHA_Q8     64        // EMA 系数 Q8: 64/256 = 0.25

// 置信度 → 增益缩放因子 (Q8 定点: 256 = 1.0x)
static const uint16_t CONF_GAIN_Q8[] = {0, 128, 192, 256};
//   conf=0: 未使用 (data_valid=0 时不会进入 control_axis)
//   conf=1: 128/256 = 0.50x  (勉强检测 → 半幅度谨慎)
//   conf=2: 192/256 = 0.75x  (基本确定 → 中幅度)
//   conf=3: 256/256 = 1.00x  (高度可信 → 全幅度)

extern volatile uint32_t sys_tick_ms;
extern uint32_t step_remain_r;   // gimbal_motor.c, 电机剩余步数

// ── 状态变量 ──
static int8_t   last_dev_x       = 0;
static uint32_t last_frame_ms    = 0;
static bool     frame_received   = false;

// EMA 滤波 (Q8.8 定点) — 仅 X 轴
static int16_t  ema_x_q8         = 0;
static bool     ema_init         = false;

// 迟滞死区
static bool     in_dead_zone_r   = false;

// 最近一次有效帧的置信度
static uint8_t  last_confidence  = 0;

// PD 控制器: 上一帧的偏差 (用于计算速度)
static int8_t   last_error_x     = 0;
static bool     pd_init          = false;


// ═══════════════════════════════════════════════════════════════
// PD 步进控制 — X 轴 → R 电机 (绳长控制)
// ═══════════════════════════════════════════════════════════════
static void control_axis(int8_t deviation, uint8_t confidence)
{
    int abs_dev = (deviation < 0) ? -deviation : deviation;

    // ── 1. 迟滞死区判断 ──
    if (in_dead_zone_r)
    {
        if (abs_dev <= VISION_DEAD_ZONE_EXIT_X)
        {
            gimbal_motor_stop(GIMBAL_MOTOR_R);
            return;
        }
        in_dead_zone_r = false;
    }
    else
    {
        if (abs_dev <= VISION_DEAD_ZONE_ENTER_X)
        {
            in_dead_zone_r = true;
            gimbal_motor_stop(GIMBAL_MOTOR_R);
            return;
        }
    }

    // ── 2. PD 控制器 ──
    // velocity > 0: 球在向右移动 (偏差在增大)
    // velocity < 0: 球在向左移动 (偏差在减小, 说明在回正)
    int velocity = deviation - last_error_x;

    // PD 输出 (Q8 定点以避免浮点)
    // output = KP * error + KD * velocity
    int32_t output = (int32_t)VISION_KP_X * deviation
                   + (int32_t)VISION_KD_X * velocity;

    // ── 3. 输出 → 目标角度 ──
    // 取绝对值计算角度, 方向由 deviation 符号决定
    int32_t abs_output = (output < 0) ? -output : output;

    // 像素 → 角度: target_angle_deg = K_ANGLE * |output|  (Q8 定点)
    uint32_t target_angle_q8 = (uint32_t)abs_output * VISION_K_ANGLE_Q8;
    uint32_t target_angle = target_angle_q8 >> 8;

    // 单帧角度上限
    if (target_angle > VISION_MAX_ANGLE_PER_FRAME)
        target_angle = VISION_MAX_ANGLE_PER_FRAME;

    // ── 4. 置信度缩放 ──
    target_angle = (target_angle * (uint32_t)CONF_GAIN_Q8[confidence]) >> 8;

    // 最小可动角度: 至少 1 度, 否则不如不动
    if (target_angle < 1)
        target_angle = 1;

    // ── 5. 设置方向 ──
    if (deviation > 0)
    {
        gimbal_motor_set_dir(GIMBAL_MOTOR_R, GIMBAL_MOTOR_DIRECTION_FORWARD);
    }
    else
    {
        gimbal_motor_set_dir(GIMBAL_MOTOR_R, GIMBAL_MOTOR_DIRECTION_REVERSE);
    }

    // ── 6. 执行步进定位 ──
    // 如果电机还在执行上一步的大角度移动, 跳过本帧避免覆写
    if (step_remain_r > 50) return;   // >50 步 = 约 3 帧, 等电机走完

    gimbal_motor_set_speed(GIMBAL_MOTOR_R, VISION_STEP_SPEED_X);
    gimbal_motor_set_angle(GIMBAL_MOTOR_R, (uint8_t)target_angle);
    // set_angle 内部已调用 gimbal_motor_start, 不需要额外调用
}


// ═══════════════════════════════════════════════════════════════
// 主入口 — 每轮主循环调用一次
// ═══════════════════════════════════════════════════════════════
void process_deviation(void)
{
    // 1. 读取最新帧 (dev_y 透传但忽略)
    uint8_t frame_status;
    int8_t dev_x, dev_y;
    if (UART_get_deviations(&frame_status, &dev_x, &dev_y))
    {
        last_frame_ms  = sys_tick_ms;
        frame_received = true;

        bool data_valid    = (frame_status & 0x01) != 0;
        uint8_t confidence = (frame_status >> 1) & 0x03;

        if (data_valid)
        {
            last_dev_x      = dev_x;
            last_confidence = confidence;

            // EMA 低通滤波 (Q8.8 定点, alpha = 64/256 = 0.25) — 仅 X 轴
            if (!ema_init)
            {
                ema_x_q8 = (int16_t)dev_x * 256;
                ema_init = true;
            }
            else
            {
                ema_x_q8 += (int16_t)(((int32_t)VISION_EMA_ALPHA_Q8
                            * ((int32_t)dev_x * 256 - (int32_t)ema_x_q8)) >> 8);
            }
        }
        else if (ema_init)
        {
            // 本帧无效 → EMA 向 0 衰减
            ema_x_q8 -= ema_x_q8 >> 2;
        }
    }

    // 2. 失联保护: 从未收到帧, 或超过 TIMEOUT_MS 无帧 → 停机 + 复位
    if (!frame_received || (sys_tick_ms - last_frame_ms) > VISION_TIMEOUT_MS)
    {
        gimbal_motor_stop(GIMBAL_MOTOR_R);
        ema_init  = false;
        pd_init   = false;
        return;
    }

    // 3. 用 EMA 滤波后的偏差值
    int8_t filtered_dev = (int8_t)(ema_x_q8 >> 8);

    // 4. 初始化 PD 历史 (首次有效帧)
    if (!pd_init)
    {
        last_error_x = filtered_dev;
        pd_init = true;
    }

    // 5. PD 步进控制 → R 电机 (绳长控制)
    control_axis(filtered_dev, last_confidence);

    // 6. 更新 PD 历史 (供下一帧计算 velocity)
    last_error_x = filtered_dev;
}
