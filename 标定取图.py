import time, os, gc
from media.sensor import *
from media.display import *
from media.media import *
#1
W, H = 640, 480                      # ← 和 AprilTag 保持一致

sensor = Sensor(width=W, height=H)
sensor.reset()
sensor.set_framesize(width=W, height=H)
sensor.set_pixformat(Sensor.RGB565)

Display.init(Display.VIRT, width=W, height=H, fps=30, to_ide=True)
sensor.run()

# 稳定阶段
for _ in range(30):
    os.exitpoint()                   # 稳定阶段也让它可中断
    img = sensor.snapshot()
    Display.show_image(img)

SAVE_EVERY = 130    # 每 N 帧存一张 —— 按你实测帧率调！
TOTAL = 30

cnt = 0
n = 0
while n < TOTAL:
    os.exitpoint()                   # ★★★ 关键：不加这个，IDE 停不下来 = 卡死 ★★★
    img = sensor.snapshot()
    Display.show_image(img)
    cnt += 1
    if cnt % SAVE_EVERY == 0:
        img.save("/sdcard/img_%02d.jpg" % n)   # 没 SD 卡换 "/data/img_%02d.jpg"
        print("saved", n)
        n += 1
    gc.collect()