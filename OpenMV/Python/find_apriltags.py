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
TAG_FAMILY   = image.TAG36H11
GOODNESS_MIN = 0.3           # filter weak detections

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
        # Filter by goodness, then pick largest (closest / most reliable)
        for tag in tags:
            if tag.goodness() < GOODNESS_MIN:
                continue
            if best_tag is None or (tag.w() * tag.h()) > (best_tag.w() * best_tag.h()):
                best_tag = tag

    if best_tag is not None:
        offset_x = best_tag.cx() - img.width() // 2
        offset_y = best_tag.cy() - img.height() // 2

        # Confidence from tag goodness
        g = best_tag.goodness()
        if g > 0.8:
            confidence = 3
        elif g > 0.5:
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

    # ── Pack and send 5-byte frame ──
    status = (data_valid << 0) | (confidence << 1)

    data = struct.pack("<BBbbB", 0xAA, status, offset_x, offset_y, 0xBB)

    if DEBUG:
        byte_sent = uart.write(data)
        if best_tag:
            print(f"Tag ID={best_tag.id()} fam={best_tag.family()} "
                  f"good={best_tag.goodness():.2f} rot={best_tag.rotation():.1f}")
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
