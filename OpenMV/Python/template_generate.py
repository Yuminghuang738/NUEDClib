import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=15000)
sensor.skip_frames(time=10000)

img = sensor.snapshot()
img.scale(x_scale=32/img.width(), y_scale=32/img.height())
img.save("template.pgm")

print("Template saved as template.pgm (32x32)")
