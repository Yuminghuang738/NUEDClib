# ============================================================
# 车载平衡滚球运动控制系统 — K230D CanMV 视觉检测模块
# ============================================================
#
# 使用 cv_lite.rgb888_find_blobs (原生 C, 等同 OpenMV find_blobs 速度)
#
# 检测策略:
#   1. cv_lite.rgb888_find_blobs + 绿色 RGB 阈值 → 水管
#   2. cv_lite.rgb888_find_blobs + 暗色 RGB 阈值 → 钢球(在水管ROI内筛选)
#   3. 启动时记录钢球初始位置作为参考零点
#   4. 每帧计算偏移量
#
# 调参: 改 GREEN_THRESHOLD / BALL_THRESHOLD, IDE 看框
# ============================================================

import time, os, sys, gc, struct
from machine import Pin
from media.sensor import *
from media.display import *
from media.media import *
import _thread
import cv_lite

# ═══════════════════════════════════════════════════════════════
# UART — 暂时注释, 先确认视觉检测正常再打开
# ═══════════════════════════════════════════════════════════════
# TODO: K230D TX=GPIO5(Pin11) → MSPM0 RX(P0)
#
# from machine import UART, FPIOA
# fpioa = FPIOA()
# fpioa.set_function(5, FPIOA.UART2_TXD)
# fpioa.set_function(6, FPIOA.UART2_RXD)
# uart = UART(2)


# ═══════════════════════════════════════════════════════════════
# 图像参数
# ═══════════════════════════════════════════════════════════════

IMG_H = 240
IMG_W = 320
IMAGE_SHAPE = [IMG_H, IMG_W]

# ═══════════════════════════════════════════════════════════════
# 绿色水管 — RGB 阈值 [Rmin,Rmax, Gmin,Gmax, Bmin,Bmax]
# ═══════════════════════════════════════════════════════════════
# 绿色物体: R 低, G 高, B 中等偏低
# 在 IDE 中观察水管 RGB 值来调
#
GREEN_THRESHOLD = [0, 80,    # R: 排除红色
                   50, 255,  # G: 绿通道要高
                   0, 100]   # B: 排除蓝色

PIPE_MIN_AREA  = 4000   # 水管 blob 最小面积
PIPE_MIN_RATIO = 2.0    # 长宽比 (过滤非长条形)
PIPE_KERNEL    = 2

# ═══════════════════════════════════════════════════════════════
# 钢球 — RGB 阈值 (暗色/金属色)
# ═══════════════════════════════════════════════════════════════
#
BALL_THRESHOLD = [0, 70,    # R: 低
                  0, 70,    # G: 低
                  0, 70]    # B: 低

BALL_MIN_AREA  = 20
BALL_KERNEL    = 1

# ═══════════════════════════════════════════════════════════════
# 其他参数
# ═══════════════════════════════════════════════════════════════

ROI_MARGIN  = 30
REF_FRAMES  = 15
DEBUG       = True

# ═══════════════════════════════════════════════════════════════
# 摄像头初始化 (RGB888 模式)
# ═══════════════════════════════════════════════════════════════

sensor = None
display_inited = False

try:
    sensor = Sensor(id=2, width=1280, height=720, fps=90)
    sensor.reset()
    sensor.set_framesize(width=IMG_W, height=IMG_H)
    sensor.set_pixformat(Sensor.RGB888)

    Display.init(Display.VIRT, width=IMG_W, height=IMG_H,
                 to_ide=True, quality=50)
    display_inited = True

    sensor.run()
    clock = time.clock()
    print("Balance Ball K230D ready. (RGB888 + cv_lite)")

    # ═══════════════════════════════════════════════════════════
    # 辅助函数
    # ═══════════════════════════════════════════════════════════

    def find_best_blob(blobs, min_area, min_ratio=0.0):
        """从 [x,y,w,h,...] 中找面积最大且满足形状条件的 blob。"""
        if blobs is None or len(blobs) < 4:
            return None

        n = len(blobs) // 4
        best = None
        best_area = 0
        for i in range(n):
            x = blobs[i * 4]
            y = blobs[i * 4 + 1]
            w = blobs[i * 4 + 2]
            h = blobs[i * 4 + 3]
            area = w * h
            if area < min_area:
                continue
            if min_ratio > 0.0:
                ratio = max(w, h) / max(min(w, h), 1)
                if ratio < min_ratio:
                    continue
            if area > best_area:
                best_area = area
                best = (x + w // 2, y + h // 2, x, y, w, h, area)
        return best


    def clamp_roi(x, y, w, h):
        nx = max(0, x)
        ny = max(0, y)
        nw = min(IMG_W - nx, w)
        nh = min(IMG_H - ny, h)
        if nw <= 0 or nh <= 0:
            return None
        return (nx, ny, nw, nh)


    def blob_in_roi(cx, cy, roi):
        rx, ry, rw, rh = roi
        return (rx <= cx <= rx + rw) and (ry <= cy <= ry + rh)

    # ═══════════════════════════════════════════════════════════
    # 全局状态
    # ═══════════════════════════════════════════════════════════

    ref_x = None
    ref_y = None
    ref_sum_x = 0
    ref_sum_y = 0
    ref_count = 0
    ref_locked = False

    # ═══════════════════════════════════════════════════════════
    # 主循环
    # ═══════════════════════════════════════════════════════════

    while True:
        os.exitpoint()
        clock.tick()
        img = sensor.snapshot()
        img_np = img.to_numpy_ref()

        # ── 第 1 步: cv_lite 找绿色水管 ──
        pipe_blobs = cv_lite.rgb888_find_blobs(
            IMAGE_SHAPE, img_np,
            GREEN_THRESHOLD,
            PIPE_MIN_AREA, PIPE_KERNEL
        )
        pipe_info = find_best_blob(pipe_blobs, PIPE_MIN_AREA, PIPE_MIN_RATIO)

        ball_cx = None
        ball_cy = None
        ball_area = 0
        ball_in_pipe = False

        if pipe_info is not None:
            pcx, pcy, px, py, pw, ph, pa = pipe_info

            if DEBUG:
                img.draw_rectangle(px, py, pw, ph,
                                   color=(0, 255, 0), thickness=2)

            # ── 第 2 步: cv_lite 找暗色钢球 ──
            ball_blobs = cv_lite.rgb888_find_blobs(
                IMAGE_SHAPE, img_np,
                BALL_THRESHOLD,
                BALL_MIN_AREA, BALL_KERNEL
            )

            roi = clamp_roi(px - ROI_MARGIN, py - ROI_MARGIN,
                            pw + 2 * ROI_MARGIN, ph + 2 * ROI_MARGIN)

            if roi is not None and ball_blobs is not None and len(ball_blobs) >= 4:
                rx, ry, rw, rh = roi
                if DEBUG:
                    img.draw_rectangle(rx, ry, rw, rh,
                                       color=(255, 255, 0), thickness=1)

                # 筛选 ROI 内面积最大的暗色 blob
                n = len(ball_blobs) // 4
                best_ba = 0
                for i in range(n):
                    bx = ball_blobs[i * 4] + ball_blobs[i * 4 + 2] // 2
                    by = ball_blobs[i * 4 + 1] + ball_blobs[i * 4 + 3] // 2
                    bw = ball_blobs[i * 4 + 2]
                    bh = ball_blobs[i * 4 + 3]
                    ba = bw * bh
                    if blob_in_roi(bx, by, roi) and ba > best_ba:
                        best_ba = ba
                        ball_cx = bx
                        ball_cy = by
                        ball_area = ba
                        ball_in_pipe = (px <= bx <= px + pw and
                                        py <= by <= py + ph)

        # 水管找不到时全图找暗色 blob 降级
        if pipe_info is None:
            ball_blobs = cv_lite.rgb888_find_blobs(
                IMAGE_SHAPE, img_np,
                BALL_THRESHOLD,
                BALL_MIN_AREA, BALL_KERNEL
            )
            best = find_best_blob(ball_blobs, BALL_MIN_AREA)
            if best is not None:
                ball_cx, ball_cy, _, _, bw, bh, ball_area = best

        # ── 第 3 步: 参考采集 / 偏差计算 ──
        ball_found = (ball_cx is not None)
        offset_x = 0
        offset_y = 0
        data_valid = 0
        confidence = 0

        if ball_found:
            # 钢球标记: 红色十字 + 圆圈
            if DEBUG:
                img.draw_cross(ball_cx, ball_cy,
                               color=(255, 0, 0), size=10, thickness=2)
                img.draw_circle(ball_cx, ball_cy, 6,
                                color=(255, 0, 0), thickness=2)

            if not ref_locked:
                ref_sum_x += ball_cx
                ref_sum_y += ball_cy
                ref_count += 1
                if DEBUG and ref_count % 5 == 0:
                    print("[REF] %d/%d" % (ref_count, REF_FRAMES))
                if ref_count >= REF_FRAMES:
                    ref_x = ref_sum_x // ref_count
                    ref_y = ref_sum_y // ref_count
                    ref_locked = True
                    if DEBUG:
                        print("[REF] locked! (%d,%d)" % (ref_x, ref_y))
            else:
                offset_x = ball_cx - ref_x
                offset_y = ball_cy - ref_y

                if DEBUG:
                    # 参考零点: 蓝色十字
                    img.draw_cross(ref_x, ref_y,
                                   color=(0, 0, 255), size=12, thickness=2)
                    img.draw_circle(ref_x, ref_y, 5,
                                    color=(0, 0, 255), thickness=1)

                # int8 钳位
                if offset_x > 79:   offset_x = 79
                elif offset_x < -80: offset_x = -80
                if offset_y > 59:   offset_y = 59
                elif offset_y < -60: offset_y = -60

                if ball_area > 100 and ball_in_pipe:
                    confidence = 3
                elif ball_area > 50:
                    confidence = 2
                elif ball_area >= BALL_MIN_AREA:
                    confidence = 1
                data_valid = 1

        # ── 第 4 步: OpenMV 5 字节协议输出 ──
        if not ref_locked:
            status = 0
            frame_off_x = 0
            frame_off_y = 0
        else:
            status = (data_valid << 0) | (confidence << 1)
            frame_off_x = offset_x if ball_found else 0
            frame_off_y = offset_y if ball_found else 0

        # TODO: UART 发送 (和 OpenMV 相同格式)
        # data = struct.pack("<BBbbB",
        #                    0xAA, status, frame_off_x, frame_off_y, 0xBB)
        # uart.write(data)

        # ── HUD ──
        if DEBUG:
            fps = clock.fps()
            # 水管中心十字
            if pipe_info is not None:
                img.draw_cross(pipe_info[0], pipe_info[1],
                               color=(0, 255, 0), size=6, thickness=1)

            if ref_locked:
                img.draw_string_advanced(
                    2, 2, 14,
                    "off=(%d,%d) c=%d" % (frame_off_x, frame_off_y, confidence),
                    color=(255, 255, 255))
            else:
                img.draw_string_advanced(
                    2, 2, 14,
                    "[REF] %d/%d" % (ref_count, REF_FRAMES),
                    color=(0, 255, 255))
            img.draw_string_advanced(
                2, IMG_H - 18, 12,
                "FPS:%d" % int(fps),
                color=(200, 200, 200))

        Display.show_image(img)
        gc.collect()

except KeyboardInterrupt:
    print("user stop")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    if display_inited:
        Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
