import time
import sensor
import image
import struct
from image import SEARCH_EX
from machine import UART
from machine import LED

# UART1, TX:P1 RX:P0
uart = UART(1, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# ── Debug switch ──
DEBUG = True

# ── Template matching params ──
TEMPLATE_PATH = "template.pgm"   # template image on SD card
RELEVANCE     = 0.6              # minimum correlation score (0.0 ~ 1.0)
STEP          = 4                # search step size (larger = faster, coarser)

# ── Sensor setup ──
sensor.set_contrast(1)
sensor.set_gainceiling(16)
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

# ── Load template (once at startup) ──
template = image.Image(TEMPLATE_PATH)

while True:
    clock.tick()
    img = sensor.snapshot()

    # ── Search for template (SEARCH_EX returns all matches above threshold) ──
    matches = img.find_template(template, RELEVANCE, step=STEP, search=SEARCH_EX)

    if matches:
        # Best match is first in the list
        best = matches[0]

        offset_x = best.cx() - img.width() // 2
        offset_y = best.cy() - img.height() // 2

        # ── Confidence based on correlation score ──
        score = best.value()   # 0.0 ~ 1.0
        if score > 0.85:
            confidence = 3
        elif score > 0.70:
            confidence = 2
        else:
            confidence = 1

        data_valid = 1

        if DEBUG:
            img.draw_rectangle(best)
            img.draw_cross(best.cx(), best.cy())
    else:
        offset_x   = 0
        offset_y   = 0
        data_valid = 0
        confidence = 0

    # ── Pack and send 5-byte frame ──
    status = (data_valid << 0) | (confidence << 1)
    # bit0 = data_valid, bit1-2 = confidence, bit3-7 = reserved

    data = struct.pack("<BBbbB", 0xAA, status, offset_x, offset_y, 0xBB)

    if DEBUG:
        byte_sent = uart.write(data)
        score_str = f"score={matches[0].value():.2f}" if matches else "score=N/A"
        print(f"Status: 0x{status:02X} valid={data_valid} conf={confidence}")
        print(f"Offset X: {offset_x}  Y: {offset_y}  {score_str}")
        print(f"FPS: {clock.fps()}")
        print(f"Sent Num: {byte_sent}")
    else:
        uart.write(data)
        red.off()
        green.toggle()
