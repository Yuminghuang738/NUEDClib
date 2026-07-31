import sensor, image, time, ml, math, uos, gc
import ml.postprocessing.edgeimpulse

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# 加载模型和标签
net = None
labels = None
min_confidence = 0.5

try:
    net = ml.Model("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    raise Exception('Failed to load "trained.tflite" (' + str(e) + ')')

try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('Failed to load "labels.txt" (' + str(e) + ')')

colors = [
    (255, 0, 0), (0, 255, 0), (255, 255, 0),
    (0, 0, 255), (255, 0, 255), (0, 255, 255), (255, 255, 255)
]

# 创建 FOMO 后处理器
fomo = ml.postprocessing.edgeimpulse.Fomo(threshold=min_confidence)

clock = time.clock()
while True:
    clock.tick()
    img = sensor.snapshot()

    # 推理并后处理
    detections = net.predict([img], callback=fomo)

    for i, detection_list in enumerate(detections):
        if i == 0 or len(detection_list) == 0:
            continue

        print("********** %s **********" % labels[i])
        for (x, y, w, h), score in detection_list:
            center_x = math.floor(x + (w / 2))
            center_y = math.floor(y + (h / 2))
            print(f"x {center_x}\ty {center_y}\tscore {score:.3f}")
            img.draw_circle((center_x, center_y, 12), color=colors[i % len(colors)])

    print(clock.fps(), "fps", end="\n\n")
