import time
from media.sensor import *
from media.display import *      # ← 1. 加这个
from media.media import *

W, H = 320, 240
sensor = Sensor(width=W, height=H)
sensor.reset()
sensor.set_framesize(width=W, height=H)
sensor.set_pixformat(Sensor.GRAYSCALE)

Display.init(Display.VIRT, width=W, height=H, fps=60, to_ide=False)  # ← 3. 初始化显示(显示到 IDE)

sensor.run()
for _ in range(30):
    img = sensor.snapshot()
    Display.show_image(img)           # 稳定阶段也显示

n = 0
while n < 30:
    img = sensor.snapshot()
    Display.show_image(img)           # ← 每帧显示(实时预览)
    img.save("/sdcard/img_%02d.jpg" % n)
    print("saved", n)
    n += 1
    time.sleep(1.5)                   # 每 1.5s 存一张，中间搬动棋盘换角度