import sensor
import time
import struct
from machine import UART
from machine import LED

# ── UART init ──
uart = UART(1, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# ── Debug switch ──
DEBUG = True

# ── Hough circle params ──
THRESHOLD = 2500          # Canny edge threshold (integer, not tuple)
X_STRIDE  = 2             # horizontal skip (pixels)
Y_STRIDE  = 1             # vertical skip (pixels)
R_MIN     = 3             # min radius (px)
R_MAX     = 50            # max radius (px)
R_STEP    = 2             # radius step (px)
MAG_MIN   = 30            # minimum accumulator magnitude

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
    img = sensor.snapshot()

    circles = img.find_circles(threshold=THRESHOLD,
                               x_margin=X_STRIDE, y_margin=Y_STRIDE,
                               r_margin=R_STEP,
                               r_min=R_MIN, r_max=R_MAX)

    best_circle = None
    if circles:
        for c in circles:
            if c.magnitude() < MAG_MIN:
                continue
            if best_circle is None or c.magnitude() > best_circle.magnitude():
                best_circle = c

    if best_circle is not None:
        offset_x = best_circle.x() - img.width() // 2
        offset_y = best_circle.y() - img.height() // 2

        # Confidence from accumulator magnitude
        mag = best_circle.magnitude()
        if mag > 80:
            confidence = 3
        elif mag > 40:
            confidence = 2
        else:
            confidence = 1

        data_valid = 1

        if DEBUG:
            img.draw_circle(best_circle.x(), best_circle.y(),
                            best_circle.r(), color=127)
            img.draw_cross(best_circle.x(), best_circle.y())
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
        mag_str = f"mag={best_circle.magnitude()}" if best_circle else "mag=N/A"
        print(f"Status: 0x{status:02X} valid={data_valid} conf={confidence}")
        print(f"Offset X: {offset_x}  Y: {offset_y}  {mag_str}")
        print(f"FPS: {clock.fps()}")
        print(f"Sent Num: {byte_sent}")
    else:
        uart.write(data)
        red.off()
        green.toggle()
