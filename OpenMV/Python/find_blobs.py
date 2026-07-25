import sensor
import time
import struct
from machine import UART
from machine import LED

# UART1,TX:P1 RX:P0
uart = UART(1, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# Debug switch
DEBUG = True

thresholds = [(0, 64)]
# dete_area: [x, y, w, h, weight]
dete_area = [(0, 80, 160, 30, 0.6), (0, 50, 160, 25, 0.3), (0, 20, 160, 25, 0.1)]
min_pixel = 100
min_area = 100

weight_sum = 0
for area in dete_area:
    weight_sum += area[4]

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=25000)
sensor.skip_frames(time=2000)

clock = time.clock()

green = LED("LED_GREEN")
red = LED("LED_RED")
red.on()

while True:
    clock.tick()
    img = sensor.snapshot()

    centroid_sum_x = 0.0
    centroid_sum_y = 0.0
    active_weight_sum = 0.0
    active_count = 0
    total_fill_ratio = 0.0

    for area in dete_area:
        blobs = img.find_blobs(thresholds, roi=area[0:4], pixels_threshold=min_pixel, area_threshold=min_area, merge=False)

        if blobs:
            # Find the blob with the most pixels
            largest_blob = max(blobs, key=lambda b: b.pixels())

            # --- Blob validity checks ---
            roi_w, roi_h = area[2], area[3]
            blob_fill_ratio = largest_blob.pixels() / (roi_w * roi_h)
            if blob_fill_ratio > 0.7:
                continue  # Blob fills too much of ROI → likely background noise

            blob_density = largest_blob.pixels() / largest_blob.area()
            if blob_density < 0.3:
                continue  # Blob too sparse → scattered noise

            if largest_blob.w() < 2 or largest_blob.h() < 2:
                continue  # Blob too thin → noise line

            # --- Blob passed all checks ---
            if DEBUG:
                # Draw a rect around the blob
                img.draw_rectangle(largest_blob.rect())
                img.draw_cross(largest_blob.cx(), largest_blob.cy())

            centroid_sum_x += largest_blob.cx() * area[4]
            centroid_sum_y += largest_blob.cy() * area[4]
            active_weight_sum += area[4]
            active_count += 1
            total_fill_ratio += blob_fill_ratio

    # --- Build status byte and compute offsets ---
    if active_weight_sum > 0:
        # Valid data: compute weighted centroid using active weights only
        center_x = int(centroid_sum_x / active_weight_sum)
        center_y = int(centroid_sum_y / active_weight_sum)
        offset_x = center_x - img.width() // 2
        offset_y = center_y - img.height() // 2

        # Confidence: 0~3 based on active ROI count and fill ratio
        avg_fill_ratio = total_fill_ratio / active_count
        if active_count >= 3 and avg_fill_ratio > 0.1:
            confidence = 3
        elif active_count >= 2:
            confidence = 2
        elif active_count >= 1:
            confidence = 1
        else:
            confidence = 0

        data_valid = 1
    else:
        # No valid blob found → invalid data
        offset_x = 0
        offset_y = 0
        data_valid = 0
        confidence = 0

    status = (data_valid << 0) | (confidence << 1)
    # status byte: bit0 = data_valid, bit1-2 = confidence, bit3-7 = reserved

    data = struct.pack("<BBbbB", 0xAA, status, offset_x, offset_y, 0xBB)
    if DEBUG:
        byte_sent = uart.write(data)

        print(f"Status: 0x{status:02X} valid={data_valid} conf={confidence}")
        print(f"Offset X: {offset_x}  Y: {offset_y}")
        print(f"FPS: {clock.fps()}")
        print(f"Sent Num:{byte_sent}")
    else:
        uart.write(data)
        red.off()
        green.toggle()
