import sensor
import time
import struct
import tf
from machine import UART
from machine import LED

# ── UART init ──
uart = UART(1, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# ── Debug switch ──
DEBUG = True

# ── Digit detection params ──
MODEL_PATH   = "num_identify.tflite"
IMG_SIZE     = 28            # model input is 28x28
THRESHOLD    = (128, 255)    # white digits on dark background
MIN_PIXELS   = 80            # min blob size for a digit candidate

# ── Sensor setup ──
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)          # 160 x 120
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=25000)
sensor.skip_frames(time=2000)

clock = time.clock()

green = LED("LED_GREEN")
red   = LED("LED_RED")
red.on()

# ── Load TFLite model (load_to_fb=True maps model I/O to frame buffer RAM) ──
net = tf.load(MODEL_PATH, load_to_fb=True)

while True:
    clock.tick()
    img = sensor.snapshot()

    # Find the largest bright blob as digit candidate
    blobs = img.find_blobs([THRESHOLD], pixels_threshold=MIN_PIXELS, merge=True)

    largest_blob = None
    if blobs:
        largest_blob = max(blobs, key=lambda b: b.pixels())

    predicted_digit = None
    nn_confidence   = 0.0

    if largest_blob is not None:
        # Crop, scale, and apply adaptive threshold
        roi = img.crop(largest_blob.rect(), copy=True)
        roi_scaled = roi.resize(IMG_SIZE, IMG_SIZE)

        # Mean adaptive threshold for robust binarisation
        roi_bin = roi_scaled.mean(2).binary([(0, 128)], invert=False)

        # Write preprocessed digit into frame buffer at (0,0)
        img.draw_image(roi_bin, 0, 0)

        # Classify
        results = net.classify(img)
        if results:
            predicted_digit = results[0].output()
            nn_confidence   = results[0].value()

        offset_x = largest_blob.cx() - img.width() // 2
        offset_y = largest_blob.cy() - img.height() // 2
    else:
        offset_x   = 0
        offset_y   = 0

    # ── Build status ──
    if largest_blob is not None and predicted_digit is not None:
        data_valid = 1

        # Map NN confidence to 1..3
        if nn_confidence > 0.9:
            confidence = 3
        elif nn_confidence > 0.6:
            confidence = 2
        else:
            confidence = 1
    else:
        data_valid = 0
        confidence = 0

    # ── Pack and send 5-byte frame ──
    status = (data_valid << 0) | (confidence << 1)

    data = struct.pack("<BBbbB", 0xAA, status, offset_x, offset_y, 0xBB)

    if DEBUG:
        byte_sent = uart.write(data)
        if predicted_digit is not None:
            print(f"Digit: {predicted_digit}  NN conf: {nn_confidence:.3f}")
        else:
            print("Digit: N/A")
        print(f"Status: 0x{status:02X} valid={data_valid} conf={confidence}")
        print(f"Offset X: {offset_x}  Y: {offset_y}")
        print(f"FPS: {clock.fps()}")
        print(f"Sent Num: {byte_sent}")
    else:
        uart.write(data)
        red.off()
        green.toggle()
