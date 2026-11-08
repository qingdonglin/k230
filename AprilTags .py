# AprilTags + UART2  发 y, tz(距离), d1(高度差)
import time, math, os, gc, sys
import struct
from machine import UART, FPIOA
from media.sensor import *
from media.display import *
from media.media import *

fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
uart = UART(UART.UART2, 115200)

DETECT_WIDTH  = 320
DETECT_HEIGHT = 240
f_x = 395.3
f_y = 395.2
c_x = 160.2
c_y = 123.6

SIZE = 0.024      # 半码长(米) = d/2
D3   = 0.009        # 码底 → 高台面 距离差(米)
D2   = 0.0419       # 光心 → 车平台面 高度差(米, 正值)

# 帧: [AA][55][y:4B][tz:4B][d1:4B][chk:1B] = 15字节
def send_frame(y, tz, d1):
    payload = struct.pack('<fff', y, tz, d1)
    chk = 0
    for b in payload:
        chk ^= b
    uart.write(bytes([0xAA, 0x55]) + payload + bytes([chk]))

def degree(r):
    return (180 * r) / math.pi
def degrees(r):
    return math.degrees(math.atan2(math.sin(r), math.cos(r)))

sensor = None
try:
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    #Display.init(Display.ST7701, to_ide=False)
    Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=60, to_ide=True)#只ide
    sensor.run()

    fps = time.clock()
    while True:
        fps.tick()
        os.exitpoint()
        img = sensor.snapshot()

        for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y):
            img.draw_rectangle([v for v in tag.rect()], color=(255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))

            tx = tag.x_translation() * SIZE          # 左右
            ty = tag.y_translation() * SIZE          # 上下(y 正=上)
            tz = -tag.z_translation() * SIZE         # 距离
            h_code = ty                              # 码高度(上为正)
            h_platform = h_code - SIZE - D3          # 平台高度
            d1 = h_platform + D2                     # 高度差

            rx = degree(tag.x_rotation()) - 180
            ry = degrees(tag.y_rotation())
            rz = -degrees(tag.z_rotation())

            send_frame(ty, tz, d1)                   # ★ 发 y, tz, d1
            print("tz:%.3f d1:%.4f h_platform:%.4f" % (tz, d1, h_platform))   # 不要就注释
            print("ID:%d  Tx:%.3f Ty:%.3f Tz:%.3f  Rx:%.2f Ry:%.2f Rz:%.2f"
                  % (tag.id(), tx, ty, tz, rx, ry, rz))
        Display.show_image(img)
        gc.collect()
        #print(fps.fps())
except KeyboardInterrupt:
    print("user stop")
except BaseException as e:
    print("Exception '%s'" % e)
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)