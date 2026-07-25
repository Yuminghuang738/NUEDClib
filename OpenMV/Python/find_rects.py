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

# ── Rectangle detection params ──
THRESHOLD  = 1000         # edge-magnitude threshold for find_rects
AREA_MIN   = 200          # min bounding-box area (px)
AREA_MAX   = 8000         # max bounding-box area (px)
MAG_MIN    = 1000         # min polygon regularity magnitude

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

    rects = img.find_rects(threshold=THRESHOLD)

    best_rect = None
    if rects:
        for r in rects:
            area = r.rect()[2] * r.rect()[3]
            if area < AREA_MIN or area > AREA_MAX:
                continue
            if r.magnitude() < MAG_MIN:
                continue
            if best_rect is None or r.magnitude() > best_rect.magnitude():
                best_rect = r

    if best_rect is not None:
        # Compute center from 4 corner points
        corners = best_rect.corners()
        cx = int(sum(p[0] for p in corners) / 4)
        cy = int(sum(p[1] for p in corners) / 4)

        offset_x = cx - img.width() // 2
        offset_y = cy - img.height() // 2

        # Confidence from polygon magnitude
        mag = best_rect.magnitude()
        if mag > 8000:
            confidence = 3
        elif mag > 4000:
            confidence = 2
        else:
            confidence = 1

        data_valid = 1

        if DEBUG:
            img.draw_rectangle(best_rect.rect())
            img.draw_cross(cx, cy)
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
        mag_str = f"mag={best_rect.magnitude()}" if best_rect else "mag=N/A"
        print(f"Status: 0x{status:02X} valid={data_valid} conf={confidence}")
        print(f"Offset X: {offset_x}  Y: {offset_y}  {mag_str}")
        print(f"FPS: {clock.fps()}")
        print(f"Sent Num: {byte_sent}")
    else:
        uart.write(data)
        red.off()
        green.toggle()
