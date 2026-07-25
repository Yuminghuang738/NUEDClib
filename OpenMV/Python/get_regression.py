import sensor
import time
import struct
import math
from machine import UART
from machine import LED

# UART1, TX:P1 RX:P0
uart = UART(1, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# ── Debug switch ──
DEBUG = True

# ── Line detection params ──
THRESHOLD      = (0, 64)       # grayscale threshold for binary image
BINARY_VISIBLE = True          # True = show binary image in IDE
MIN_MAGNITUDE  = 20            # minimum regression magnitude (pixels in line)

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

while True:
    clock.tick()

    # ── Snapshot (optionally binarize for debug view) ──
    if BINARY_VISIBLE:
        img = sensor.snapshot().binary([THRESHOLD])
    else:
        img = sensor.snapshot()

    # ── Linear regression line detection ──
    line = img.get_regression([(255, 255) if BINARY_VISIBLE else THRESHOLD])

    if line is not None and line.magnitude() >= MIN_MAGNITUDE:
        # ── Compute offset from line-segment midpoint ──
        x1, y1, x2, y2 = line.line()
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2

        offset_x = center_x - img.width() // 2
        offset_y = center_y - img.height() // 2

        # ── Confidence based on magnitude (line "strength") ──
        mag = line.magnitude()
        if mag > 80:
            confidence = 3
        elif mag > 40:
            confidence = 2
        else:
            confidence = 1

        data_valid = 1

        if DEBUG:
            img.draw_line(line.line(), color=127)
            img.draw_cross(center_x, center_y)
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
        mag_str = f"mag={line.magnitude()}" if line is not None else "mag=N/A"
        print(f"Status: 0x{status:02X} valid={data_valid} conf={confidence}")
        print(f"Offset X: {offset_x}  Y: {offset_y}  {mag_str}")
        print(f"FPS: {clock.fps()}")
        print(f"Sent Num: {byte_sent}")
    else:
        uart.write(data)
        red.off()
        green.toggle()
