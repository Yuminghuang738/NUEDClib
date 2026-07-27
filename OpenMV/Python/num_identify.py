"""
Handwritten digit recognition for OpenMV.

Assumption: dark digit on light background (pen on paper).
MNIST format: white digit (255) on black background (0).
"""
import sensor
import image
import time
import ml

# ── Sensor setup ──
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)             # 320 x 240
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=25000)
sensor.skip_frames(time=2000)

clock = time.clock()

# ── Load model ──
# NO load_to_fb — we pass the 28×28 ROI directly to avoid stride mismatch
MODEL_PATH = "num_identify.tflite"
net = ml.Model(MODEL_PATH)

# ── Tunable constants ──
THRESH_FRAC  = 0.70   # fraction of 60th-percentile used as dark/light cutoff
MIN_BLOB_PX  = 50     # ignore blobs smaller than this
ROI_PAD      = 0.20   # expand ROI by 20 % for MNIST-style margin

print("Ready. Show a dark digit on light background.")

while True:
    clock.tick()
    img = sensor.snapshot()

    # ── 1. Adaptive threshold from frame histogram ──
    hist = img.get_histogram()
    hist_stats = hist.statistics()         # (mean, median, mode, stdev, ...)
    bg_ref  = hist_stats[1]                 # median — roughly the paper brightness        # roughly the paper brightness
    thresh = int(bg_ref * THRESH_FRAC)
    thresh = max(60, min(180, thresh))        # clamp to safe range

    # ── 2. Find the largest dark blob (the digit) ──
    blobs = img.find_blobs([(0, thresh)], pixels_threshold=MIN_BLOB_PX, merge=True)

    if blobs:
        largest = max(blobs, key=lambda b: b.pixels())
        r = largest.rect()                    # (x, y, w, h)

        # ── 3. Expand ROI with padding ──
        pw = int(r[2] * ROI_PAD)
        ph = int(r[3] * ROI_PAD)
        rx = max(0, r[0] - pw)
        ry = max(0, r[1] - ph)
        rw = min(img.width()  - rx, r[2] + 2 * pw)
        rh = min(img.height() - ry, r[3] + 2 * ph)

        # ── 4. Crop → scale to 28×28 → binary ──
        roi = img.copy(roi=(rx, ry, rw, rh))
        roi = roi.copy(x_scale=28.0 / rw, y_scale=28.0 / rh,
                       hint=image.BILINEAR)

        # Recompute threshold on ROI for better foreground / background split
        roi_hist = roi.get_histogram()
        roi_hist_stats = roi_hist.statistics()
        roi_bg  = roi_hist_stats[1]
        roi_thresh = int(roi_bg * 0.70)
        roi_thresh = max(40, min(160, roi_thresh))

        # invert=False: pixels in [0, thresh] → 255 (white digit),
        #               pixels outside       → 0   (black bg)
        # → white-digit-on-black-bg = MNIST format
        roi.binary([(0, roi_thresh)], invert=False)

        # ── 5. Morphological CLOSE — dilate then erode fills gaps in strokes ──
        roi.dilate(1, threshold=128)
        roi.erode(1, threshold=128)

        # ── 6. Predict directly from ROI (no draw_image, no stride issues) ──
        results = net.predict([roi])

        if results:
            row = results[0][0]
            best_idx = 0
            best_val = row[0]
            for i in range(1, 10):
                if row[i] > best_val:
                    best_val = row[i]
                    best_idx = i

            # ── Diagnostic output ──
            prob_str = " ".join(["%d:%.2f" % (i, row[i]) for i in range(10)])

            roi_stats = roi.get_histogram().statistics()
            # statistics() → (mean, median, mode, stdev, min, max, lq, uq)

            print("─" * 50)
            print("Probs: %s" % prob_str)
            print("-> Digit: %d  (conf: %.2f)  FPS: %.1f"
                  % (best_idx, best_val, clock.fps()))
            print("  ROI px [min:%d max:%d mean:%d]  thresh:%d/%d"
                  % (roi_stats[4], roi_stats[5], int(roi_stats[0]),
                     thresh, roi_thresh))

            img.draw_string(largest.cx(), largest.cy() + r[3] // 2 + 10,
                            str(best_idx), color=255, scale=3)
            img.draw_rectangle(r, color=255)
    else:
        if clock.fps() < 1:
            print(".", end="")
