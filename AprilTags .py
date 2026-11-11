# AprilTags + UART2 发 左右, 距离, d1(高度差)
# 640x480 | 精确式: h = ty*cosθ + tz*sinθ  (D 已联立消掉，无近似)
# ------------------------------------------------------------------
# 两个标定常数：
#   SIN_TH : 相机歪角正弦。相机姿态锁死，前后平移 5 点，
#            拟合 ty 对 D 的斜率即可。当前 0.0513 来自实测数据。
#   D2     : 零点偏移。平齐时看打印的"D2建议"，填进去。
# ------------------------------------------------------------------
import time, math, os, gc
import struct
from machine import UART, FPIOA
from media.sensor import *
from media.display import *
from media.media import *

fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
uart = UART(UART.UART2, 115200)

DETECT_WIDTH  = 640
DETECT_HEIGHT = 480
f_x = 790.62
f_y = 790.41
c_x = 320.36
c_y = 247.18

SIZE = 0.024      # 半码长(米)，源码确认
D3   = 0.010      # 码底 → 头部平台面
D2   =  0.0538    # 零点偏移：平齐时 D2 = 0.034 - h  （待实测重标）

# ===== 相机歪角标定 =====
SIN_TH = 0.0688
COS_TH = math.sqrt(1.0 - SIN_TH * SIN_TH)     # = 0.99868

# ===== 其它 =====
TAG_ID = 0
TZ_MIN, TZ_MAX = 0.10, 0.90
ALPHA = 0.25
MEDIAN_N = 5
PRINT_EVERY = 6
EDGE_MARGIN = 0


def send_frame(lat, dist, d1):
    payload = struct.pack('<fff', lat, dist, d1)
    chk = 0
    for b in payload:
        chk ^= b
    uart.write(bytes([0xAA, 0x55]) + payload + bytes([chk]))


sensor = None
try:
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=60, to_ide=True)
    sensor.run()

    fps = time.clock()
    d1_f = None
    d1_buf = []
    n = 0

    while True:
        fps.tick()
        os.exitpoint()
        img = sensor.snapshot()
        n += 1

        best = None
        best_a = 0
        for t in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y):
            if TAG_ID is not None and t.id() != TAG_ID:
                continue
            r = t.rect()
            a = r[2] * r[3]
            if a > best_a:
                best_a, best = a, t

        if best is not None:
            r = best.rect()
            in_view = (r[0] >= EDGE_MARGIN and r[1] >= EDGE_MARGIN and
                       r[0] + r[2] <= DETECT_WIDTH - EDGE_MARGIN and
                       r[1] + r[3] <= DETECT_HEIGHT - EDGE_MARGIN)
            if in_view:
                tx = best.x_translation() * SIZE
                ty = best.y_translation() * SIZE
                tz = best.z_translation() * SIZE
                dist_raw = math.sqrt(tx * tx + ty * ty + tz * tz)

                if TZ_MIN <= dist_raw <= TZ_MAX:

                    # ===== 核心：两式联立，精确解 =====
                    #   ty =  D·sinθ + h·cosθ
                    #   tz = −D·cosθ + h·sinθ
                    #   => ①×cos + ②×sin 得 h；①×sin − ②×cos 得 D
                    h = ty * COS_TH + tz * SIN_TH        # 码中心相对相机的高度
                    D = ty * SIN_TH - tz * COS_TH        # 水平距离

                    h_platform = h - SIZE - D3
                    d1_raw = h_platform + D2

                    # 5 点中值 + EMA
                    d1_buf.append(d1_raw)
                    if len(d1_buf) > MEDIAN_N:
                        d1_buf.pop(0)
                    med = sorted(d1_buf)[len(d1_buf) // 2]
                    if d1_f is None:
                        d1_f = med
                    else:
                        d1_f = ALPHA * med + (1.0 - ALPHA) * d1_f

                    send_frame(tx, D, d1_f)

                    img.draw_rectangle([r[0], r[1], r[2], r[3]], color=(255, 0, 0))
                    img.draw_cross(best.cx(), best.cy(), color=(0, 255, 0))

                    if n % PRINT_EVERY == 0:
                        print("D:%.3f ty:%+.4f h:%+.4f d1f:%+.4f | D2建议:%.4f" %
                              (D, ty, h, d1_f, SIZE + D3 - h))

        Display.show_image(img)
        if n % 100 == 0:
            gc.collect()

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