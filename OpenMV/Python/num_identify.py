import sensor
import time
import image
import ml

# ── Sensor setup ──
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)            # 320 x 240
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=25000)
sensor.skip_frames(time=2000)

clock = time.clock()

# ── Load model ──
MODEL_PATH = "num_identify.tflite"
net = ml.Model(MODEL_PATH, load_to_fb=True)

print("Ready. Show a digit to the camera.")

while True:
    clock.tick()
    img = sensor.snapshot()

    # Find the largest white blob
    blobs = img.find_blobs([(128, 255)], pixels_threshold=80, merge=True)

    if blobs:
        largest = max(blobs, key=lambda b: b.pixels())
        r = largest.rect()

        # Crop → scale 28x28 → binary threshold
        roi = img.copy(roi=r)
        x_scale = 28.0 / r[2]
        y_scale = 28.0 / r[3]
        roi = roi.copy(x_scale=x_scale, y_scale=y_scale, hint=image.BILINEAR)
        roi.binary([(0, 128)], invert=False)

        # Write to frame buffer (model input) and classify
        img.draw_image(roi, 0, 0)
        results = net.predict([img])

        if results:
            row = results[0][0]
            best_idx = 0
            best_val = row[0]
            for i in range(1, 10):
                if row[i] > best_val:
                    best_val = row[i]
                    best_idx = i

            # Print digit on serial terminal
            print("Digit: %d  (conf: %.2f)  FPS: %.1f" % (best_idx, best_val, clock.fps()))

            # Draw digit on the camera image (IDE preview)
            img.draw_string(largest.cx(), largest.cy() + r[3]//2 + 5,
                            str(best_idx), color=255, scale=3)
            img.draw_rectangle(r, color=255)
