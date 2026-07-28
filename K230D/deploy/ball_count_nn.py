# ============================================================
# 钢球数量识别 — K230D CanMV (庐山派 Lite-K230D-CanMV)
# Steel Ball Counter using cv_lite blob detection
# ============================================================

import time, os, sys, gc
from machine import Pin
from media.sensor import *     # 摄像头
from media.display import *    # 显示
from media.media import *      # 媒体管理
import _thread
import cv_lite                 # blob 检测
import ulab.numpy as np        # 轻量 numpy

# -------------------------------
# 图像参数
# -------------------------------
image_shape = [480, 640]       # [height, width]

# -------------------------------
# blob 检测参数
# -------------------------------
threshold  = [180, 255]        # 高光阈值 [min, max]
min_area   = 6                 # 最小 blob 面积
kernel_size = 1                # 腐蚀核

# -------------------------------
# KPU
# -------------------------------
MODEL_PATH = "/sdcard/model.kmodel"
IMG_SIZE   = 128         # 模型输入尺寸 (与训练时一致)

sensor = None

try:
    # ---------------------------
    # 初始化摄像头
    # ---------------------------
    sensor = Sensor(id=2, width=1280, height=720, fps=90)
    sensor.reset()
    sensor.set_framesize(width=image_shape[1], height=image_shape[0])
    sensor.set_pixformat(Sensor.GRAYSCALE)

    # ---------------------------
    # 初始化显示 (IDE 虚拟显示)
    # ---------------------------
    Display.init(Display.VIRT, width=image_shape[1], height=image_shape[0],
                 to_ide=True, quality=50)

    # ---------------------------
    # 尝试加载 KPU 模型
    # ---------------------------
    kpu = None
    try:
        files = os.listdir("/sdcard")
        if "model.kmodel" in files:
            import nncase_runtime as nn
            kpu = nn.kpu()
            kpu.load_kmodel(MODEL_PATH)
            print("KPU model loaded")
    except Exception as e:
        print("KPU:", e)

    mode_str = "KPU" if kpu else "CV"
    print(f"Steel Ball Counter ready. Mode: {mode_str}")

    sensor.run()
    clock = time.clock()

    # ---------------------------
    # 主循环
    # ---------------------------
    while True:
        clock.tick()
        img = sensor.snapshot()

        if kpu:
            # KPU 神经网络推理
            try:
                import nncase_runtime as nn

                side = min(image_shape[1], image_shape[0])
                cx = (image_shape[1] - side) // 2
                cy = (image_shape[0] - side) // 2

                ai2d = nn.ai2d()
                ai2d.set_dtype(nn.ai2d_format.NCHW_FMT,
                               nn.ai2d_format.NCHW_FMT,
                               np.uint8, np.float32)
                ai2d.set_crop_param(
                    True, [cx, cy, cx + side - 1, cy + side - 1])
                ai2d.set_resize_param(
                    True,
                    nn.interp_method.tf_bilinear,
                    nn.interp_mode.half_pixel)
                builder = ai2d.build(
                    [1, 1, image_shape[0], image_shape[1]],
                    [1, 1, IMG_SIZE, IMG_SIZE])

                kpu.set_input_tensor(0,
                    nn.from_numpy(img.to_numpy_ref()))
                kpu.run()

                out = kpu.get_output_tensor(0).to_numpy().flatten()
                out = out - out.max()
                exp = np.exp(out)
                probs = exp / exp.sum()
                count = int(np.argmax(probs))

                del ai2d, builder
            except Exception as e:
                print("KPU error:", e)
                kpu = None
                count = 0

        if not kpu:
            # cv_lite blob 检测
            img_np = img.to_numpy_ref()
            blobs = cv_lite.grayscale_find_blobs(
                image_shape, img_np,
                threshold[0], threshold[1],
                min_area, kernel_size
            )

            count = 0
            n = len(blobs) // 4
            for i in range(n):
                x = int(blobs[i * 4])
                y = int(blobs[i * 4 + 1])
                w = int(blobs[i * 4 + 2])
                h = int(blobs[i * 4 + 3])

                area = w * h
                if area > 300:
                    continue
                if area < min_area:
                    continue
                aspect = max(w, 1) / max(h, 1)
                if aspect > 3.0 or aspect < 0.33:
                    continue

                count += 1
                img.draw_rectangle(x, y, w, h,
                                   color=(255, 255, 255), thickness=2)
                img.draw_cross(x + w // 2, y + h // 2,
                               color=(255, 255, 255), size=4, thickness=1)

        # HUD
        fps = clock.fps()
        img.draw_string_advanced(4, 4, 24, f"Balls: {count}",
                                 color=(255, 255, 255))
        img.draw_string_advanced(4, 30, 16,
                                 f"FPS:{fps:.0f} [{mode_str}]",
                                 color=(200, 200, 200))

        Display.show_image(img)
        print(f"Balls: {count}  FPS: {fps:.1f}  [{mode_str}]")
        gc.collect()

except KeyboardInterrupt:
    print("user stop")
except Exception as e:
    sys.print_exception(e)
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
