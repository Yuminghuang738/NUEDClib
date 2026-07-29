"""
车载平衡滚球运动控制系统 — OpenMV 视觉检测模块
=================================================

检测策略:
  1. 通过绿色 LAB 阈值找到水管区域 (ROI)
  2. 在 ROI 内通过暗色/中性色阈值找到钢球
  3. 启动时记录钢球初始位置作为平衡参考零点
  4. 每帧计算钢球偏离初始位置的偏差, 通过 5 字节 UART 协议发送

硬件连接:
  OpenMV TX(P1) → MSPM0 RX(P0), 115200bps

协议 (与现有框架兼容):
  [0xAA] [status] [offset_x:int8] [offset_y:int8] [0xBB]
  status: bit0=data_valid, bit1-2=confidence(0~3)

调参顺序:
  1. 先调 GREEN_THRESHOLD 确保水管完整框出 (IDE 中观察)
  2. 再调 BALL_THRESHOLD 确保钢球在水管内被准确检出
  3. 调整 ROI_MARGIN 确保钢球在管道边缘时也不丢失
  4. 最后调整 REF_FRAMES 控制初始位置采样帧数
"""

import sensor
import time
import struct
from machine import UART
from machine import LED

# ═══════════════════════════════════════════════════════════════
# UART 初始化
# ═══════════════════════════════════════════════════════════════

uart = UART(1, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# ═══════════════════════════════════════════════════════════════
# Debug 开关 — 比赛时改为 False 以减少串口打印开销
# ═══════════════════════════════════════════════════════════════

DEBUG = True

# ═══════════════════════════════════════════════════════════════
# 绿色水管 LAB 阈值 — 需根据实际水管颜色现场调参
# ═══════════════════════════════════════════════════════════════
#
# LAB 颜色空间:
#   L: 0=黑 ~ 100=白   (亮度)
#   A: -128=绿 ~ +127=红
#   B: -128=蓝 ~ +127=黄
#
# 典型绿色 PVC 水管: L 中等, A 偏负(绿), B 近零或略正
# 调参方法: 在 IDE 中拍一帧水管照片, 用阈值编辑器拖拽滑块
#
GREEN_THRESHOLD = [
(0, 100, -128, -10, -128, 127)
]

# ═══════════════════════════════════════════════════════════════
# 钢球 LAB 阈值 — 钢球呈暗色/金属色, A/B 接近中性
# ═══════════════════════════════════════════════════════════════
#
# 钢球特征: 暗灰色金属, A≈0, B≈0
# 注意: 如果钢球反光严重导致检测破裂, 可放宽 L 上界
#       或尝试将 BALL_USE_FIND_CIRCLES 设为 True 用霍夫圆检测
#
BALL_THRESHOLD = [
(0, 100, -6, 127, -128, 127)
]

# 备选: 用霍夫圆检测替代颜色 blob (OpenMV IDE 固件 >= 4.0)
# 如果颜色阈值法误检/漏检多, 可改用圆检测
BALL_USE_FIND_CIRCLES = False

# ═══════════════════════════════════════════════════════════════
# 滤波 & ROI 参数
# ═══════════════════════════════════════════════════════════════

PIPE_MIN_PIXELS  = 300    # 水管最小像素数 (小于此值视为未检测到水管)
PIPE_MIN_DENSITY = 0.25   # 水管 blob 最小密度 (过滤稀疏噪点)

BALL_MIN_PIXELS  = 20     # 钢球最小像素数 (远距离时可能需要调小)
BALL_MIN_DENSITY = 0.35   # 钢球 blob 最小密度 (钢球应为较密实的 blob)

ROI_MARGIN = 15           # 水管 ROI 外扩像素 (确保球在管道边缘时不被裁掉)

# ═══════════════════════════════════════════════════════════════
# 初始参考位置采集 — 启动时钢球应处于自然平衡位置
# ═══════════════════════════════════════════════════════════════

REF_FRAMES = 15           # 采集帧数 (取平均作为参考零点)

# ═══════════════════════════════════════════════════════════════
# 霍夫圆检测参数 (仅在 BALL_USE_FIND_CIRCLES=True 时使用)
# ═══════════════════════════════════════════════════════════════

CIRCLE_THRESHOLD = 2000   # Canny 边缘阈值
CIRCLE_R_MIN     = 2      # 最小半径 (px)
CIRCLE_R_MAX     = 30     # 最大半径 (px)
CIRCLE_R_STEP    = 2      # 半径步进
CIRCLE_MAG_MIN   = 20     # 最小累加器强度

# ═══════════════════════════════════════════════════════════════
# 传感器初始化
# ═══════════════════════════════════════════════════════════════

sensor.reset()
sensor.set_pixformat(sensor.RGB565)           # 彩色模式 (LAB 阈值依赖)
sensor.set_framesize(sensor.QQVGA)            # 160×120 (协议偏移范围硬依赖)
sensor.set_auto_gain(False)                   # 必须关闭, 否则阈值漂移
sensor.set_auto_whitebal(False)               # 必须关闭
sensor.set_auto_exposure(False, exposure_us=20000)
sensor.skip_frames(time=2000)

clock = time.clock()

green_led = LED("LED_GREEN")
red_led   = LED("LED_RED")
red_led.on()

# ═══════════════════════════════════════════════════════════════
# 全局状态
# ═══════════════════════════════════════════════════════════════

ref_x = None                # 锁定后的参考 X
ref_y = None                # 锁定后的参考 Y
ref_sum_x = 0               # 累积 X (用于取平均)
ref_sum_y = 0               # 累积 Y
ref_count = 0               # 已采集的有效帧数
ref_locked = False          # 参考是否已锁定

# ═══════════════════════════════════════════════════════════════
# 辅助函数
# ═══════════════════════════════════════════════════════════════

def clamp_roi(x, y, w, h, img_w, img_h):
    """将 ROI 裁剪到图像边界内, 返回 (x, y, w, h) 或 None (如果完全出界)"""
    if x >= img_w or y >= img_h or x + w <= 0 or y + h <= 0:
        return None
    nx = max(0, x)
    ny = max(0, y)
    nw = min(img_w - nx, x + w - nx)
    nh = min(img_h - ny, y + h - ny)
    return (nx, ny, nw, nh)


def find_largest_valid_blob(img, thresholds, roi=None,
                             min_pixels=20, min_density=0.3):
    """
    在 (可选 ROI) 内找到最大且密度达标的 blob。
    返回 blob 对象或 None。
    """
    if roi is not None:
        blobs = img.find_blobs(thresholds, roi=roi,
                               pixels_threshold=min_pixels,
                               area_threshold=min_pixels,
                               merge=True)
    else:
        blobs = img.find_blobs(thresholds,
                               pixels_threshold=min_pixels,
                               area_threshold=min_pixels,
                               merge=True)
    best = None
    for b in blobs:
        density = b.pixels() / max(b.area(), 1)
        if density < min_density:
            continue
        if best is None or b.pixels() > best.pixels():
            best = b
    return best


def compute_confidence(ball_blob, pipe_blob):
    """
    根据钢球检测质量和相对水管位置计算置信度 (0~3)。

    标准:
      conf=3: 钢球像素充足、密度高、在水管区域内
      conf=2: 钢球基本可靠、在水管区域内或附近
      conf=1: 钢球勉强检出、信号弱
      conf=0: 未检测到
    """
    if ball_blob is None:
        return 0

    pixels = ball_blob.pixels()
    density = ball_blob.pixels() / max(ball_blob.area(), 1)

    # 检查球心是否在水管 ROI 内 (如果水管可见)
    ball_in_pipe = True
    if pipe_blob is not None:
        pr = pipe_blob.rect()
        bx, by = ball_blob.cx(), ball_blob.cy()
        ball_in_pipe = (pr[0] - ROI_MARGIN <= bx <= pr[0] + pr[2] + ROI_MARGIN and
                        pr[1] - ROI_MARGIN <= by <= pr[1] + pr[3] + ROI_MARGIN)

    if pixels > 80 and density > 0.5 and ball_in_pipe:
        return 3
    elif pixels > 40 and density > 0.4:
        return 2
    elif pixels >= BALL_MIN_PIXELS:
        return 1
    else:
        return 0


def find_ball_in_roi(img, roi_tuple):
    """
    在给定 ROI 内寻找钢球。
    根据 BALL_USE_FIND_CIRCLES 选择 blob 或霍夫圆检测。
    roi_tuple 为 None 时全图搜索。

    返回: (cx, cy, extras) 或 (None, None, None)
      extras: blob 对象 (blob模式) 或 circle 对象 (圆检测模式)
    """
    if BALL_USE_FIND_CIRCLES:
        if roi_tuple is not None:
            sub = img.copy(roi=roi_tuple)
        else:
            sub = img.copy()  # 全图
        circles = sub.find_circles(
            threshold=CIRCLE_THRESHOLD,
            x_margin=2, y_margin=1,
            r_margin=CIRCLE_R_STEP,
            r_min=CIRCLE_R_MIN, r_max=CIRCLE_R_MAX
        )
        best = None
        for c in circles:
            if c.magnitude() < CIRCLE_MAG_MIN:
                continue
            if best is None or c.magnitude() > best.magnitude():
                best = c
        if best is not None:
            # 转回全图坐标 (全图时 roi 偏移为 0)
            ox = roi_tuple[0] if roi_tuple is not None else 0
            oy = roi_tuple[1] if roi_tuple is not None else 0
            return (ox + best.x(), oy + best.y(), best)
        return (None, None, None)
    else:
        blob = find_largest_valid_blob(
            img, BALL_THRESHOLD, roi=roi_tuple,
            min_pixels=BALL_MIN_PIXELS, min_density=BALL_MIN_DENSITY
        )
        if blob is not None:
            return (blob.cx(), blob.cy(), blob)
        return (None, None, None)


# ═══════════════════════════════════════════════════════════════
# 主循环
# ═══════════════════════════════════════════════════════════════

while True:
    clock.tick()
    img = sensor.snapshot()

    # ── 第 1 步: 找到绿色水管, 确定搜索 ROI ──
    pipe_blob = find_largest_valid_blob(
        img, GREEN_THRESHOLD,
        min_pixels=PIPE_MIN_PIXELS, min_density=PIPE_MIN_DENSITY
    )

    ball_cx = None
    ball_cy = None
    ball_extra = None
    pipe_roi = None

    if pipe_blob is not None:
        # 从水管外接矩形扩展出钢球搜索 ROI
        pr = pipe_blob.rect()
        roi_x = pr[0] - ROI_MARGIN
        roi_y = pr[1] - ROI_MARGIN
        roi_w = pr[2] + 2 * ROI_MARGIN
        roi_h = pr[3] + 2 * ROI_MARGIN

        pipe_roi = clamp_roi(roi_x, roi_y, roi_w, roi_h,
                             img.width(), img.height())

        if DEBUG:
            img.draw_rectangle(pipe_blob.rect(), color=(0, 255, 0))  # 绿框标水管

        # ── 第 2 步: 在水管 ROI 内找钢球 ──
        if pipe_roi is not None:
            if DEBUG:
                # 用虚线风格标记搜索 ROI (OpenMV 不支持虚线, 用细框替代)
                img.draw_rectangle(pipe_roi, color=(255, 255, 0))  # 黄框标搜索区域

            ball_cx, ball_cy, ball_extra = find_ball_in_roi(img, pipe_roi)
    else:
        # 水管未检测到 → 全图搜索钢球作为降级策略
        # (仅在管道暂时丢失但球仍可见时有用, 置信度会降低)
        ball_cx, ball_cy, ball_extra = find_ball_in_roi(img, None)

    # ── 第 3 步: 参考位置采集 / 偏差计算 ──
    ball_found = (ball_cx is not None)

    # 初始化为安全默认值 (参考采集阶段或丢球时使用)
    offset_x   = 0
    offset_y   = 0
    data_valid = 0
    confidence = 0

    if ball_found:
        # 调试绘制
        if DEBUG:
            img.draw_cross(ball_cx, ball_cy, color=(255, 0, 0))  # 红十字标钢球
            if ball_extra is not None:
                if hasattr(ball_extra, 'rect'):
                    # blob 模式: 画出钢球边界框
                    img.draw_rectangle(ball_extra.rect(), color=(255, 0, 0))
                elif hasattr(ball_extra, 'x') and hasattr(ball_extra, 'r'):
                    # 圆检测模式: 画出圆
                    img.draw_circle(ball_extra.x(), ball_extra.y(),
                                    ball_extra.r(), color=(255, 0, 0))

        if not ref_locked:
            # ── 参考采集阶段: 累积钢球位置 ──
            ref_sum_x += ball_cx
            ref_sum_y += ball_cy
            ref_count += 1

            if DEBUG and ref_count % 5 == 0:
                print(f"[REF] collecting... {ref_count}/{REF_FRAMES}")

            if ref_count >= REF_FRAMES:
                ref_x = ref_sum_x // ref_count
                ref_y = ref_sum_y // ref_count
                ref_locked = True
                if DEBUG:
                    print(f"[REF] locked! ref_x={ref_x}, ref_y={ref_y}")
        else:
            # ── 正常跟踪阶段: 计算偏离参考位置的偏差 ──
            offset_x = ball_cx - ref_x
            offset_y = ball_cy - ref_y

            if DEBUG and ref_locked:
                # 画出参考位置十字 (蓝色)
                img.draw_cross(ref_x, ref_y, color=(0, 0, 255), size=8)

            # 边界检查 (int8 范围)
            if offset_x > 79:
                offset_x = 79
            elif offset_x < -80:
                offset_x = -80
            if offset_y > 59:
                offset_y = 59
            elif offset_y < -60:
                offset_y = -60

            confidence = compute_confidence(
                ball_extra if (ball_extra is not None and hasattr(ball_extra, 'pixels')) else None,
                pipe_blob
            )
            data_valid = 1
    else:
        # 钢球未检测到 — offset/data_valid/confidence 已默认为 0
        pass

    # ── 第 4 步: 打包并发送 5 字节帧 ──
    # 如果在参考采集阶段且钢球未检测到, 仍发送无效帧以维持 MSPM0 连接存活
    if not ref_locked:
        # 参考未锁定期间: 始终发送 data_valid=0 (MSPM0 侧会保持停机/超时状态)
        status = 0
        frame_offset_x = 0
        frame_offset_y = 0
    else:
        status = (data_valid << 0) | (confidence << 1)
        # bit0 = data_valid, bit1-2 = confidence, bit3-7 = reserved
        frame_offset_x = offset_x if ball_found else 0
        frame_offset_y = offset_y if ball_found else 0

    data = struct.pack("<BBbbB",
                       0xAA, status, frame_offset_x, frame_offset_y, 0xBB)

    if DEBUG:
        byte_sent = uart.write(data)
        if ref_locked:
            print(f"offset_x={frame_offset_x:4d}  offset_y={frame_offset_y:4d}  "
                  f"valid={data_valid}  conf={confidence}  "
                  f"ref=({ref_x},{ref_y})  fps={clock.fps():.1f}")
        else:
            print(f"[REF] collecting {ref_count}/{REF_FRAMES}  fps={clock.fps():.1f}")
    else:
        uart.write(data)
        red_led.off()
        green_led.toggle()
