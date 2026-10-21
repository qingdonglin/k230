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
f_x = 395.3   # 焦距x（像素）
f_y = 395.2   # 焦距y（像素）
c_x = 160.2   # 光心x（像素）≈ 画面中心
c_y = 123.6   # 光心y（像素）≈ 画面中心
SIZE = 0.024  # 半码长（米）          


#二进制帧打包函数 `send_frame`
# ============================================================
# ② 发送一帧: [AA][55][y:4字节][distance:4字节][校验:1字节] = 11字节
#     - y / distance 都是"小端float"(4字节)
#     - 校验 = 中间8个数据字节依次异或
#     - 顺序: 先 y, 后 distance (电控端按同样顺序解)
# ============================================================
def send_frame(y, distance):
    payload = struct.pack('<ff', y, distance)   # 小端, 两个float -> 8字节
    #校验字节，对 **payload 中间 8 个数据字节**逐个异或
    chk = 0
    for b in payload:                           # 8字节异或
        chk ^= b
    uart.write(bytes([0xAA, 0x55]) + payload + bytes([chk])) #最终发出

def degrees(r):
    # atan2 归一化到 ±180（务必用这个版，不要再出现 271°）
    return math.degrees(math.atan2(math.sin(r), math.cos(r)))

sensor = None
try:
    #传感器初始化
    sensor = Sensor(width = DETECT_WIDTH, height = DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width = DETECT_WIDTH, height = DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)

    Display.init(Display.VIRT, width = DETECT_WIDTH, height = DETECT_HEIGHT, fps = 60, to_ide = True)
    sensor.run()

    #主循环  核心检测与发送
    fps = time.clock()#fps.tick() 打时间戳用于算帧率；`os.exitpoint()` 响应 IDE 停止信号
    while True:
        fps.tick()
        os.exitpoint()
        img = sensor.snapshot()#抓一张图

        for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y):#`img.find_apriltags(...)` 在图里找所有标签，返回 `tag` 对象列表
            img.draw_rectangle([v for v in tag.rect()], color=(255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))

            tx = tag.x_translation() * SIZE     # 左右(米)
            ty = tag.y_translation() * SIZE     # 上下(米) -> y
            tz = -tag.z_translation() * SIZE     # 距离(米) -> distance
            # 旋转 → 度（本来就是真实角度，不用乘）
            rx = degrees(tag.x_rotation())      # 绕 x 轴旋转角(°)
            ry = degrees(tag.y_rotation())      # 绕 y 轴旋转角(°)
            rz = degrees(tag.z_rotation())      # 绕 z 轴旋转角(°)

            
            # ===== 发送: 先 y 后 distance（二进制+校验）=====
            send_frame(ty, tz)

            # 打印标签 ID、平移(m) 与旋转角(°)
            print("ID:%d  Tx:%.3f Ty:%.3f Tz:%.3f  Rx:%.2f Ry:%.2f Rz:%.2f"
                  % (tag.id(), tx, ty, tz, rx, ry, rz))

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