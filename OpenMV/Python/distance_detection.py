import sensor
import time
import struct
import image
from machine import UART
from machine import LED

# ── UART init ──
uart = UART(1, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# ── Debug switch ──
DEBUG = True

# ── AprilTag params ──
TAG_FAMILY       = image.TAG36H11
TAG_REAL_SIZE_MM = 30.0       # physical side length of the tag (mm)
FOCAL_LEN        = 150.0       # calibrated focal length (px) — tune this!
GOODNESS_MIN     = 0.3

# ── Calibration note ──
# Place a TAG_REAL_SIZE_MM tag at a known distance D (mm).
# Read the tag's pixel width w from the debug output.
# Compute:  FOCAL_LEN = (D * w) / TAG_REAL_SIZE_MM
# Replace the FOCAL_LEN value above with your calibrated result.

# ── Sensor setup ──
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)          # 160 x 120
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=15000)
sensor.skip_frames(time=2000)

clock = time.clock()

green = LED("LED_GREEN")
red   = LED("LED_RED")
red.on()

while True:
    clock.tick()
    img = sensor.snapshot()

    tags = img.find_apriltags(families=TAG_FAMILY)

    best_tag = None
    if tags:
        for tag in tags:
            if tag.goodness() < GOODNESS_MIN:
                continue
            if best_tag is None or (tag.w() * tag.h()) > (best_tag.w() * best_tag.h()):
                best_tag = tag

    distance_mm = 0.0

    if best_tag is not None:
        offset_x = best_tag.cx() - img.width() // 2
        offset_y = best_tag.cy() - img.height() // 2

        # Monocular ranging: d = (focal_length * real_size) / pixel_size
        # Use tag width as the pixel measurement
        pixel_size = best_tag.w()
        if pixel_size > 0:
            distance_mm = (FOCAL_LEN * TAG_REAL_SIZE_MM) / pixel_size

        # Confidence from goodness and proximity
        g = best_tag.goodness()
        if g > 0.7:
            confidence = 3
        elif g > 0.4:
            confidence = 2
        else:
            confidence = 1

        data_valid = 1

        if DEBUG:
            img.draw_rectangle(best_tag.rect())
            img.draw_cross(best_tag.cx(), best_tag.cy())
    else:
        offset_x   = 0
        offset_y   = 0
        data_valid = 0
        confidence = 0

    # ── Pack and send 5-byte frame (distance is debug-only, not in protocol) ──
    status = (data_valid << 0) | (confidence << 1)

    data = struct.pack("<BBbbB", 0xAA, status, offset_x, offset_y, 0xBB)

    if DEBUG:
        byte_sent = uart.write(data)
        if best_tag:
            print(f"Tag ID={best_tag.id()} good={best_tag.goodness():.2f} "
                  f"w={best_tag.w()}px  dist={distance_mm:.1f}mm")
        else:
            print("Tag N/A")
        print(f"Status: 0x{status:02X} valid={data_valid} conf={confidence}")
        print(f"Offset X: {offset_x}  Y: {offset_y}")
        print(f"FPS: {clock.fps()}")
        print(f"Sent Num: {byte_sent}")
    else:
        uart.write(data)
        red.off()
        green.toggle()
