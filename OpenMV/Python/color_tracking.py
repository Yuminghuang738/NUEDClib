import sensor
import time
import struct
from machine import UART
from machine import LED

# UART1, TX:P1 RX:P0
uart = UART(1, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# ── Debug switch ──
DEBUG = True

# ── Red color threshold (LAB) ──
#   L: 30~80   (moderate brightness, avoids over/under-exposed)
#   A: 20~127  (positive = red, tune lower bound to exclude weak red)
#   B: -20~40  (near neutral, avoids orange/yellow bias)
# Tune these values if the target red object is not detected reliably.
thresholds = [(30, 80, 20, 127, -20, 40)]

# ── Blob filter params ──
min_pixel = 200       # minimum pixel count
min_area  = 200       # minimum bounding-box area
min_density = 0.3     # minimum pixel/area ratio (filters sparse noise)

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

    # ── Find all red blobs in frame ──
    blobs = img.find_blobs(thresholds, pixels_threshold=min_pixel,
                           area_threshold=min_area, merge=True)

    largest_blob = None

    for blob in blobs:
        # ── Density check: filter sparse / scattered noise ──
        blob_density = blob.pixels() / blob.area()
        if blob_density < min_density:
            continue

        # ── Track the largest valid blob ──
        if largest_blob is None or blob.pixels() > largest_blob.pixels():
            largest_blob = blob

    # ── Compute offsets and build status ──
    if largest_blob is not None:
        offset_x = largest_blob.cx() - img.width() // 2
        offset_y = largest_blob.cy() - img.height() // 2

        # Confidence based on blob pixel count relative to frame
        total_pixels = img.width() * img.height()
        fill_ratio = largest_blob.pixels() / total_pixels
        if fill_ratio > 0.05:
            confidence = 3
        elif fill_ratio > 0.02:
            confidence = 2
        else:
            confidence = 1

        data_valid = 1

        if DEBUG:
            img.draw_rectangle(largest_blob.rect())
            img.draw_cross(largest_blob.cx(), largest_blob.cy())
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
        print(f"Status: 0x{status:02X} valid={data_valid} conf={confidence}")
        print(f"Offset X: {offset_x}  Y: {offset_y}")
        print(f"FPS: {clock.fps()}")
        print(f"Sent Num: {byte_sent}")
    else:
        uart.write(data)
        red.off()
        green.toggle()
