# AprilTags + UART2 二进制帧发送(带校验)  先 y, 后 distance
import time, math, os, gc, sys
import struct                              # ★ 二进制打包
from machine import UART, FPIOA
from media.sensor import *
from media.display import *
from media.media import *

# ============================================================
# ① UART2 初始化（GPIO11=TX, GPIO12=RX）
# ============================================================
fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)   # GPIO11 -> TX2
fpioa.set_function(12, FPIOA.UART2_RXD)   # GPIO12 -> RX2
uart = UART(UART.UART2, 115200)

DETECT_WIDTH  = 320
DETECT_HEIGHT = 240
f_x = 395.3
f_y = 395.2
c_x = 160.2
c_y = 123.6
SIZE = 0.024                              # 半码长(米)

# ============================================================
# ② 发送一帧: [AA][55][y:4字节][distance:4字节][校验:1字节] = 11字节
#     - y / distance 都是"小端float"(4字节)
#     - 校验 = 中间8个数据字节依次异或
#     - 顺序: 先 y, 后 distance (电控端按同样顺序解)
# ============================================================
def send_frame(y, distance):
    payload = struct.pack('<ff', y, distance)   # 小端, 两个float -> 8字节
    chk = 0
    for b in payload:                           # 8字节异或
        chk ^= b
    uart.write(bytes([0xAA, 0x55]) + payload + bytes([chk]))

def degrees(radians):
    return (180 * radians) / math.pi

sensor = None
try:
    sensor = Sensor(width = DETECT_WIDTH, height = DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width = DETECT_WIDTH, height = DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)

    Display.init(Display.VIRT, width = DETECT_WIDTH, height = DETECT_HEIGHT, fps = 60, to_ide = True)
    sensor.run()

    fps = time.clock()
    while True:
        fps.tick()
        os.exitpoint()
        img = sensor.snapshot()

        for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y):
            img.draw_rectangle([v for v in tag.rect()], color=(255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))

            tx = tag.x_translation() * SIZE     # 左右(米)
            ty = tag.y_translation() * SIZE     # 上下(米) -> y
            tz = tag.z_translation() * SIZE     # 距离(米) -> distance

            # ===== 发送: 先 y 后 distance（二进制+校验）=====
            send_frame(ty, tz)

            print("ID:%d  Tx:%.3f Ty:%.3f Tz:%.3f" % (tag.id(), tx, ty, tz))

        Display.show_image(img)
        gc.collect()
        print(fps.fps())
except KeyboardInterrupt as e:
    print("user stop")
except BaseException as e:
    print("Exception '%s'" % e)
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)