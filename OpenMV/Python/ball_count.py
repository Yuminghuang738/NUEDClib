"""
Steel ball counter for OpenMV H7 Plus (firmware 4.8.1).

Strategy:
    Each steel ball creates a bright specular highlight (reflection of the
    light source).  We find those highlights with blob detection, filter out
    false positives, and count the rest — one highlight ≈ one ball.

Setup requirements:
    • Balls on a dark, matte surface (black cloth / foam / rubber)
    • Diffuse overhead lighting — a desk lamp or the room ceiling light
    • Camera mounted above, pointing straight down

How to tune:
    1. Run the script and point the camera at an empty surface
       → blob count should be 0.  If not, raise BLOB_MIN or lower exposure.
    2. Add one ball — you should see it drawn with a rectangle.
    3. Add more balls — each should get its own rectangle.
    4. If two balls touching merge into one blob, decrease MERGE_MARGIN.
    5. If a single ball splits into two blobs, increase MERGE_MARGIN.
"""

import sensor
import image
import time

# ── Sensor ──
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)            # highlights are luminance features
sensor.set_framesize(sensor.QVGA)                 # 320 × 240
sensor.set_auto_gain(False)                       # fixed = repeatable thresholds
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=20000)
sensor.skip_frames(time=2000)

clock = time.clock()

# ── Detection parameters (tune these for your lighting / surface) ──
BLOB_MIN      = 180     # min pixel brightness for a ball highlight  (0-255)
BLOB_MAX      = 255     # max pixel brightness (255 = pure white)
MIN_AREA      = 6       # ignore blobs smaller than this  (px²  — noise)
MAX_AREA      = 300     # ignore blobs larger  than this  (px²  — glare)
MIN_DENSITY   = 0.25    # 0-1, filters irregular / elongated shapes
MERGE_MARGIN  = 5       # px, higher → merge nearby blobs into one

print("Steel ball counter ready.")
print("Params:  BLOB_MIN=%d  MIN_AREA=%d  MAX_AREA=%d"
      % (BLOB_MIN, MIN_AREA, MAX_AREA))

while True:
    clock.tick()
    img = sensor.snapshot()

    # ── 1. Find bright blobs ──
    blobs = img.find_blobs(
        [(BLOB_MIN, BLOB_MAX)],
        pixels_threshold=MIN_AREA,
        merge=True,
        margin=MERGE_MARGIN,
    )

    # ── 2. Filter & count ──
    ball_count = 0
    for b in blobs:
        area = b.area()

        if area > MAX_AREA:          # surface glare, window reflection …
            continue
        if b.density() < MIN_DENSITY:  # not compact → likely a scratch or edge
            continue

        ball_count += 1

        # Draw on IDE preview
        img.draw_rectangle(b.rect(), color=255)
        img.draw_cross(b.cx(), b.cy(), color=255, size=6)

    # ── 3. Serial output ──
    print("Balls: %d   FPS: %.1f" % (ball_count, clock.fps()))
