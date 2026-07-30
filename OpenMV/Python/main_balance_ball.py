"""
车载平衡滚球运动控制系统 — OpenMV 视觉检测模块
=================================================

检测策略 (霍夫圆 + 高光融合):
  1. 通过绿色 LAB 阈值找到水管区域 (ROI)
  2. 在 ROI 内同时运行:
     a. 霍夫圆检测 → 候选圆 (几何特征: 钢球是圆形)
     b. 高光 blob 检测 → 亮斑列表 (光学特征: 钢球表面反光)
  3. 融合评分: 圆内包含高光 → 高分 (几何+光学双重确认)
     孤立圆 (magnitude 高) 或孤立高光 → 降低
  4. 启动时记录钢球初始位置作为平衡参考零点
  5. 仅发送 X 轴偏移 (沿水管长度方向), Y 轴恒为 0

硬件连接:
  OpenMV TX(P1) → MSPM0 RX(P0), 115200bps

协议:
  [0xAA] [status] [offset_x:int8] [offset_y:int8恒0] [0xBB]
  status: bit0=data_valid, bit1-2=confidence(0~3)

调参顺序:
  1. 先调 GREEN_THRESHOLD 确保水管完整框出 (IDE 中观察)
  2. 再调 CIRCLE_* 和高光参数确保钢球被准确检出
  3. 调整 ROI_MARGIN 确保钢球在管道边缘时也不丢失
  4. 最后调整 REF_FRAMES 控制初始位置采样帧数
"""

import sensor
import time
import struct
import math
from machine import UART
from machine import LED

# ═══════════════════════════════════════════════════════════════
# UART 初始化
# ═══════════════════════════════════════════════════════════════

uart = UART(1, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# ═══════════════════════════════════════════════════════════════
# Debug 开关 — 比赛时改为 False
# ═══════════════════════════════════════════════════════════════

DEBUG = True

# ═══════════════════════════════════════════════════════════════
# 绿色水管 LAB 阈值
# ═══════════════════════════════════════════════════════════════

GREEN_THRESHOLD = [
    (0, 90, -127, -15, -30, 127)
]
# L: 20-90  (排除过暗阴影/过曝反光)
# A: -128~-15 (严格绿色方向)
# B: -30~60 (限制蓝黄范围, 排除天空/墙面误检)

# ═══════════════════════════════════════════════════════════════
# 霍夫圆检测参数 — 钢球几何特征
# ═══════════════════════════════════════════════════════════════
# 钢球直径 1cm, QQVGA (160×120) 下约 8~12 px (视摄像头高度而定)

CIRCLE_THRESHOLD = 1600    # Canny 边缘阈值 (降低以检测更弱的边缘)
CIRCLE_R_MIN     = 2       # 最小半径 (px)
CIRCLE_R_MAX     = 10      # 最大半径 (px) — 1cm 球对应 ~4-6px 半径
CIRCLE_R_STEP    = 1       # 半径步进 (更细粒度)
CIRCLE_MAG_MIN   = 6       # 最小累加器强度 (小球 magnitude 天然低)

# ═══════════════════════════════════════════════════════════════
# 高光 blob 检测参数 — 钢球表面反光
# ═══════════════════════════════════════════════════════════════
# LAB 格式: (L_min, L_max, A_min, A_max, B_min, B_max)
# 钢球高光 = 亮白 (高 L, A/B 中性)

HIGHLIGHT_THRESHOLD = [(85, 100, -30, 30, -30, 30)]

# ═══════════════════════════════════════════════════════════════
# 颜色 blob 检测参数 — 钢球本体 (暗色金属, 用作兜底)
# ═══════════════════════════════════════════════════════════════

BALL_THRESHOLD = [
    (85, 100, -30, 30, -30, 30)
]
# L: 0-80   (暗色, 钢球吸收光线)
# A: -10~10 (中性色, 非绿非红)
# B: -20~20 (中性色, 非蓝非黄)
BALL_MIN_PIXELS  = 6       # 钢球本体 blob 最小像素 (1cm 球 ~80px²)
BALL_MIN_DENSITY = 0.25    # 钢球本体 blob 最小密度

# ═══════════════════════════════════════════════════════════════
# 水管 & ROI 参数
# ═══════════════════════════════════════════════════════════════

PIPE_MIN_PIXELS  = 300     # 水管最小像素数
PIPE_MIN_DENSITY = 0.25    # 水管 blob 最小密度
ROI_MARGIN       = 10      # 水管 ROI 外扩像素 (覆盖管道两端余量)

# ═══════════════════════════════════════════════════════════════
# 初始参考位置采集
# ═══════════════════════════════════════════════════════════════

REF_FRAMES = 15            # 采集帧数 (取平均作为参考零点)

# ═══════════════════════════════════════════════════════════════
# 传感器初始化
# ═══════════════════════════════════════════════════════════════

sensor.reset()
sensor.set_pixformat(sensor.RGB565)           # 彩色模式 (LAB 阈值 + 霍夫圆均需)
sensor.set_framesize(sensor.QVGA)              # 320×240, 球~10-20px, 60fps
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=25000)  # 进光量↑, 高光更亮
sensor.skip_frames(time=2000)

clock = time.clock()

green_led = LED("LED_GREEN")
red_led   = LED("LED_RED")
red_led.on()

# ═══════════════════════════════════════════════════════════════
# 全局状态
# ═══════════════════════════════════════════════════════════════

ref_x = None
ref_y = None                # 保留采集但不再用于控制
ref_sum_x = 0
ref_sum_y = 0
ref_count = 0
ref_locked = False

# 目标偏移 (像素) — 赛题任务 3/6 需要球停在非零点
# 正值=球应在参考点右侧, 负值=左侧
# 换算: QVGA下 水管 25cm ≈ 画面~260px, 所以 5cm ≈ 52px
TARGET_SHIFT = 0             # 默认 0 = 平衡在参考点

_dbg_frame = 0               # 诊断打印帧计数器
last_ball_cx = None
last_ball_cy = None
TRACK_WINDOW  = 30          # 追踪窗口半边长 (px)

# ═══════════════════════════════════════════════════════════════
# 辅助函数
# ═══════════════════════════════════════════════════════════════

def clamp_roi(x, y, w, h, img_w, img_h):
    """将 ROI 裁剪到图像边界内"""
    if x >= img_w or y >= img_h or x + w <= 0 or y + h <= 0:
        return None
    nx = max(0, x)
    ny = max(0, y)
    nw = min(img_w - nx, x + w - nx)
    nh = min(img_h - ny, y + h - ny)
    return (nx, ny, nw, nh)


def find_largest_valid_blob(img, thresholds, roi=None,
                             min_pixels=20, min_density=0.3):
    """在 (可选 ROI) 内找到最大且密度达标的 blob"""
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
    if blobs is not None:
        for b in blobs:
            density = b.pixels() / max(b.area(), 1)
            if density < min_density:
                continue
            if best is None or b.pixels() > best.pixels():
                best = b
    return best


def detect_ball_fusion(img, roi_tuple, ref_cx=None, ref_cy=None):
    """
    三通道投票检测钢球:

      通道 1 (非绿色): 二值化反相, 水管内不是绿色的东西
      通道 2 (暗色):   BALL_THRESHOLD, 钢球本体是暗色金属
      通道 3 (高光+圆): HIGHLIGHT_THRESHOLD + 霍夫圆, 钢球反光+圆形边界

    评分 = 基础分 + 通道间互相确认加分:
      - 单通道命中:            基础 1.5 分 (弱)
      - 两通道在附近都命中:     6.0~7.0  (可靠)
      - 三通道全命中:           8.0~9.0  (极高置信度)
      - 靠近上一帧位置:         +1.0

    返回: (cx, cy, score, extra) 或 (None, None, 0, None)
    """
    if roi_tuple is None:
        return (None, None, 0, None)

    BALL_PX_MIN = 4
    BALL_PX_MAX = 60
    BALL_EXPECTED_PX = 15
    ox = roi_tuple[0]
    oy = roi_tuple[1]

    # ═══════════════════════════════════════════════════════════
    # 通道 1: "非绿色" blob (二值化反相)
    # ═══════════════════════════════════════════════════════════
    not_green_candidates = []   # [(cx, cy, pixels, blob)]
    try:
        sub = img.copy(roi=roi_tuple)
        sub.binary([GREEN_THRESHOLD[0]], invert=True)
        raw = sub.find_blobs([(128, 255)],
                             pixels_threshold=BALL_PX_MIN,
                             area_threshold=BALL_PX_MIN, merge=True)
        if raw is not None:
            for b in raw:
                if BALL_PX_MIN <= b.pixels() <= BALL_PX_MAX:
                    not_green_candidates.append(
                        (ox + b.cx(), oy + b.cy(), b.pixels(), b))
    except Exception:
        pass

    # ═══════════════════════════════════════════════════════════
    # 通道 2: 暗色 blob (BALL_THRESHOLD)
    # ═══════════════════════════════════════════════════════════
    dark_candidates = []        # [(cx, cy, pixels, blob)]
    raw_dark = img.find_blobs(
        BALL_THRESHOLD, roi=roi_tuple,
        pixels_threshold=BALL_MIN_PIXELS,
        area_threshold=BALL_MIN_PIXELS, merge=True
    )
    if raw_dark is not None:
        for b in raw_dark:
            if (BALL_PX_MIN <= b.pixels() <= BALL_PX_MAX and
                b.pixels() / max(b.area(), 1) >= BALL_MIN_DENSITY):
                dark_candidates.append((b.cx(), b.cy(), b.pixels(), b))

    # ═══════════════════════════════════════════════════════════
    # 通道 3: 高光 + 霍夫圆
    # ═══════════════════════════════════════════════════════════
    hl_candidates = []          # [(cx, cy, pixels, blob)]
    raw_highlights = img.find_blobs(
        HIGHLIGHT_THRESHOLD, roi=roi_tuple,
        pixels_threshold=2, area_threshold=2, merge=False
    )
    if raw_highlights is not None:
        for h in raw_highlights:
            if h.area() <= 60 and h.density() >= 0.3:
                hl_candidates.append((h.cx(), h.cy(), h.pixels(), h))

    circle_candidates = []      # [(cx, cy, r, mag, obj)]
    try:
        sub2 = img.copy(roi=roi_tuple)
        raw_circles = sub2.find_circles(
            threshold=CIRCLE_THRESHOLD,
            x_margin=2, y_margin=1,
            r_margin=CIRCLE_R_STEP,
            r_min=CIRCLE_R_MIN, r_max=CIRCLE_R_MAX
        )
        if raw_circles is not None:
            for c in raw_circles:
                if c.magnitude() >= CIRCLE_MAG_MIN and CIRCLE_R_MIN <= c.r() <= CIRCLE_R_MAX:
                    circle_candidates.append(
                        (ox + c.x(), oy + c.y(), c.r(), c.magnitude(), c))
    except Exception:
        pass

    # ═══════════════════════════════════════════════════════════
    # 三通道投票评分
    # ═══════════════════════════════════════════════════════════
    # 每个候选位置 (candidates pool) 合并三个通道,
    # 按"附近有多少通道同时命中"评分

    # 收集所有候选点 (cx, cy, source_mask, extra_info)
    # source_mask: bit0=非绿色, bit1=暗色, bit2=高光+圆
    all_candidates = []

    for (cx, cy, px, b) in not_green_candidates:
        all_candidates.append((cx, cy, 1, b, px))

    for (cx, cy, px, b) in dark_candidates:
        all_candidates.append((cx, cy, 2, b, px))

    # 高光+圆: 只在高光靠近圆时才作为通道3候选
    for (hcx, hcy, hpx, hb) in hl_candidates:
        for (ccx, ccy, cr, cmag, cb) in circle_candidates:
            dist = int(math.sqrt((hcx - ccx) ** 2 + (hcy - ccy) ** 2))
            if dist < max(cr, 5):
                all_candidates.append((hcx, hcy, 4, hb, hpx))
                break
        else:
            # 孤立高光也加入(低权重)
            all_candidates.append((hcx, hcy, 4, hb, hpx))
    # 孤立圆
    for (ccx, ccy, cr, cmag, cb) in circle_candidates:
        all_candidates.append((ccx, ccy, 4, cb, 0))

    # ── 合并附近候选, 累加通道 mask ──
    MERGE_DIST = 15  # 15px 内视为同一目标

    merged = []  # [(avg_cx, avg_cy, channel_mask, best_extra, max_px)]
    used = [False] * len(all_candidates)

    for i, (cx1, cy1, mask1, ext1, px1) in enumerate(all_candidates):
        if used[i]:
            continue
        sum_x, sum_y, count = cx1, cy1, 1
        merged_mask = mask1
        best_ext = ext1
        max_px = px1
        used[i] = True

        for j in range(i + 1, len(all_candidates)):
            if used[j]:
                continue
            cx2, cy2, mask2, ext2, px2 = all_candidates[j]
            if abs(cx1 - cx2) < MERGE_DIST and abs(cy1 - cy2) < MERGE_DIST:
                sum_x += cx2
                sum_y += cy2
                count += 1
                merged_mask |= mask2
                if px2 > max_px:
                    max_px = px2
                    best_ext = ext2
                used[j] = True

        avg_cx = sum_x // count
        avg_cy = sum_y // count
        merged.append((avg_cx, avg_cy, merged_mask, best_ext, max_px))

    # ── 评分 ──
    best_cx, best_cy = None, None
    best_score = 0.0
    best_extra = None

    for (cx, cy, mask, ext, px) in merged:
        # 基础分: 有几个独立通道命中
        channel_count = bin(mask).count('1')
        # 通道3(高光+圆)权重高于通道1和2
        has_hl_circle = (mask & 4) != 0

        if channel_count >= 3:
            score = 8.0
        elif channel_count == 2:
            score = 6.0
        else:
            score = 1.5

        # 高光+圆命中额外加分
        if has_hl_circle:
            score += 1.0

        # 大小匹配
        px_diff = abs(px - BALL_EXPECTED_PX) if px > 0 else 99
        if px_diff < 5:
            score += 1.0

        # 追踪连续性
        if ref_cx is not None:
            dist = int(math.sqrt((cx - ref_cx) ** 2 + (cy - ref_cy) ** 2))
            if dist < TRACK_WINDOW:
                score += 1.0

        if score > best_score:
            best_score = score
            best_cx, best_cy = cx, cy
            best_extra = ext

    if DEBUG:
        global _dbg_frame
        _dbg_frame += 1
        if _dbg_frame % 30 == 1:
            print(f"  [DETECT] ng={len(not_green_candidates)} "
                  f"dk={len(dark_candidates)} "
                  f"hl={len(hl_candidates)} cir={len(circle_candidates)} "
                  f"→ merged={len(merged)} score={best_score:.1f}")

    if best_score > 0 and best_cx is not None:
        return (best_cx, best_cy, best_score, best_extra)

    return (None, None, 0, None)


def compute_confidence(score):
    """score 0~10 → confidence 0~3"""
    if score >= 8.0:
        return 3    # 三通道全命中
    elif score >= 6.0:
        return 2    # 两通道命中
    elif score >= 1.5:
        return 1    # 单通道命中
    else:
        return 0


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
    ball_score = 0.0
    ball_extra = None
    pipe_roi = None

    if pipe_blob is not None:
        pr = pipe_blob.rect()
        roi_x = pr[0] - ROI_MARGIN
        roi_y = pr[1] - ROI_MARGIN
        roi_w = pr[2] + 2 * ROI_MARGIN
        roi_h = pr[3] + 2 * ROI_MARGIN

        pipe_roi = clamp_roi(roi_x, roi_y, roi_w, roi_h,
                             img.width(), img.height())

        if DEBUG:
            img.draw_rectangle(pipe_blob.rect(), color=(0, 255, 0))

        # ── 第 2 步: 融合检测钢球 ──
        if pipe_roi is not None:
            if DEBUG:
                img.draw_rectangle(pipe_roi, color=(255, 255, 0))

            # 管道矩形 (用于几何约束)
            px, py, pw, ph = pr[0], pr[1], pr[2], pr[3]

            # 优先: 在上一帧球心附近搜索 (追踪窗口)
            if last_ball_cx is not None:
                track_roi = clamp_roi(
                    last_ball_cx - TRACK_WINDOW,
                    last_ball_cy - TRACK_WINDOW,
                    TRACK_WINDOW * 2,
                    TRACK_WINDOW * 2,
                    img.width(), img.height()
                )
                if track_roi is not None:
                    tcx, tcy, tscore, textra = detect_ball_fusion(
                        img, track_roi, last_ball_cx, last_ball_cy)
                    # 追踪窗口也必须过几何约束, 防止追踪点"走"出管道
                    # 两端放宽容差 (12px), 水管 green blob 可能未覆盖到最末端
                    if (tcx is not None and
                        px - 12 <= tcx <= px + pw + 12 and
                        py - 5  <= tcy <= py + ph + 5):
                        ball_cx, ball_cy, ball_score, ball_extra = tcx, tcy, tscore, textra

            # 追踪窗口没找到或未通过几何约束 → 回退到全管道 ROI 搜索
            if ball_cx is None:
                ball_cx, ball_cy, ball_score, ball_extra = detect_ball_fusion(
                    img, pipe_roi, last_ball_cx, last_ball_cy)

                # 几何约束 (两端 12px 容差)
                if ball_cx is not None:
                    if not (px - 12 <= ball_cx <= px + pw + 12 and
                            py - 5  <= ball_cy <= py + ph + 5):
                        ball_cx, ball_cy, ball_score, ball_extra = None, None, 0, None

            # 更新或重置追踪点
            if ball_cx is not None:
                last_ball_cx, last_ball_cy = ball_cx, ball_cy
            else:
                last_ball_cx, last_ball_cy = None, None
    else:
        last_ball_cx, last_ball_cy = None, None
    # 水管丢失 → 直接判定无球 (不降级到全图搜索, 避免假阳性)

    # ── 第 3 步: 参考位置采集 / 偏差计算 ──
    ball_found = (ball_cx is not None)

    # 安全默认值
    offset_x   = 0
    offset_y   = 0
    data_valid = 0
    confidence = 0

    if ball_found:
        if DEBUG:
            # 画检测结果
            img.draw_cross(ball_cx, ball_cy, color=(255, 0, 0), size=10)
            if ball_extra is not None:
                if hasattr(ball_extra, 'rect'):
                    img.draw_rectangle(ball_extra.rect(), color=(255, 0, 0))
                if hasattr(ball_extra, 'r'):
                    img.draw_circle(ball_cx, ball_cy, ball_extra.r(),
                                    color=(255, 0, 0))

        if not ref_locked:
            # ── 参考采集阶段 ──
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
            # ── 正常跟踪: 仅 X 轴偏差 (沿水管方向) ──
            offset_x = (ball_cx - (ref_x + TARGET_SHIFT)) // 2  # QVGA→int8
            # offset_y 恒为 0 — 水管横向放置, Y 轴不可控

            if DEBUG:
                img.draw_cross(ref_x, ref_y, color=(0, 0, 255), size=8)

            # int8 边界检查
            if offset_x > 79:
                offset_x = 79
            elif offset_x < -80:
                offset_x = -80

            confidence = compute_confidence(ball_score)
            data_valid = 1 if confidence > 0 else 0
    else:
        # 钢球未检测到 — 默认值 (0, 0)
        pass

    # ── 第 4 步: 打包并发送 5 字节帧 ──
    if not ref_locked:
        status = 0
        frame_offset_x = 0
        frame_offset_y = 0
    else:
        status = (data_valid << 0) | (confidence << 1)
        frame_offset_x = offset_x if ball_found else 0
        frame_offset_y = 0   # Y 轴恒为 0

    data = struct.pack("<BBbbB",
                       0xAA, status, frame_offset_x, frame_offset_y, 0xBB)

    if DEBUG:
        byte_sent = uart.write(data)
        if ref_locked:
            print(f"offset_x={frame_offset_x:4d}  "
                  f"valid={data_valid}  conf={confidence}  "
                  f"score={ball_score:.1f}  "
                  f"ref=({ref_x},{ref_y})  fps={clock.fps():.1f}")
        else:
            print(f"[REF] collecting {ref_count}/{REF_FRAMES}  fps={clock.fps():.1f}")
    else:
        uart.write(data)
        red_led.off()
        green_led.toggle()
