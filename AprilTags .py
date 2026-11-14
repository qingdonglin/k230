# ==================================================================
#  AprilTag + UART2 —— 定时打印全套数据版
# ==================================================================
#  【打印内容】每 PRINT_EVERY 帧打一次，分两行：
#
#    #120 tx=-0.0012 ty=-0.0023 tz=-0.2460 uup=(+0.01,-1.00,-0.07)
#         D=0.2455 h=-0.0160 hp=-0.0500 | d1=+0.0000 d1f=+0.0000
#         | D2建议=0.0500 fps=22.3
#
#    第 1 行 = PnP 原始输出（乘 SIZE 后的米值）+ 当前实测姿态
#    第 2 行 = 派生量：距离 / 高度 / 平台高度 / 高度差 / 理想D2 / 帧率
#
#  【两个标定常数】
#    SIN_TH : 相机歪角正弦（前后移动拟合 ty 对 D 的斜率）
#    D2     : 零点偏移（平齐时 D2 = SIZE + D3 − h）
# ==================================================================

import time, math, os, gc
import struct
from machine import UART, FPIOA
from media.sensor import *
from media.display import *
from media.media import *

# ---------------- 串口 ----------------
fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
uart = UART(UART.UART2, 115200)

# ---------------- 分辨率与内参 ----------------
DETECT_WIDTH  = 640
DETECT_HEIGHT = 480
f_x = 790.62
f_y = 790.41
c_x = 320.36
c_y = 247.18

# ---------------- 尺寸常量 ----------------
SIZE = 0.024      # 半码长(米)
D3   = 0.010      # 码底 → 头部平台面
D2   = 0.0497  # 零点偏移（按"建议D2"重标后填）

# ---------------- 相机歪角 ----------------
SIN_TH = 0.0552
COS_TH = math.sqrt(1.0 - SIN_TH * SIN_TH)

# ---------------- 其它 ----------------
TAG_ID = 0
TZ_MIN, TZ_MAX = 0.08, 0.90
ALPHA = 0.25          # EMA 系数
MEDIAN_N = 7          # 中值窗口
PRINT_EVERY = 1      # 每多少帧打印一次（60 ≈ 每 2~3 秒一行）
EDGE_MARGIN = 0


# ---------------- UART 帧 ----------------
# [AA][55] + <lat:4B> + <D:4B> + <d1:4B> + [XOR] = 15 字节
def send_frame(lat, dist, d1):
    payload = struct.pack('<fff', lat, dist, d1)
    chk = 0
    for b in payload:
        chk ^= b
    uart.write(bytes([0xAA, 0x55]) + payload + bytes([chk]))


# ---------------- 旋转矩阵（只为打印 uup 用） ----------------
def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _mm(A, B):
    return ((A[0][0]*B[0][0] + A[0][1]*B[1][0] + A[0][2]*B[2][0],
             A[0][0]*B[0][1] + A[0][1]*B[1][1] + A[0][2]*B[2][1],
             A[0][0]*B[0][2] + A[0][1]*B[1][2] + A[0][2]*B[2][2]),
            (A[1][0]*B[0][0] + A[1][1]*B[1][0] + A[1][2]*B[2][0],
             A[1][0]*B[0][1] + A[1][1]*B[1][1] + A[1][2]*B[2][1],
             A[1][0]*B[0][2] + A[1][1]*B[1][2] + A[1][2]*B[2][2]),
            (A[2][0]*B[0][0] + A[2][1]*B[1][0] + A[2][2]*B[2][0],
             A[2][0]*B[0][1] + A[2][1]*B[1][1] + A[2][2]*B[2][1],
             A[2][0]*B[0][2] + A[2][1]*B[1][2] + A[2][2]*B[2][2]))


def build_R(rx, ry, rz):
    """R = Rz·Ry·Rx，R 的列 = 标签各轴在相机系中的方向"""
    return _mm(_mm(_rot_z(rz), _rot_y(ry)), _rot_x(rx))


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

        # ---------- 找面积最大的目标标签 ----------
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
                # ===== PnP 原始输出（米） =====
                tx = best.x_translation() * SIZE
                ty = best.y_translation() * SIZE
                tz = best.z_translation() * SIZE
                dist_raw = math.sqrt(tx * tx + ty * ty + tz * tz)

                if TZ_MIN <= dist_raw <= TZ_MAX:

                    # ===== 精确解（两式联立，D 已消掉） =====
                    #   ty =  D·sinθ + h·cosθ
                    #   tz = −D·cosθ + h·sinθ
                    h = ty * COS_TH + tz * SIN_TH      # 码中心相对相机的高度
                    D = ty * SIN_TH - tz * COS_TH      # 水平距离

                    h_platform = h - SIZE - D3
                    d1_raw = h_platform + D2

                    # ===== 中值 + EMA =====
                    d1_buf.append(d1_raw)
                    if len(d1_buf) > MEDIAN_N:
                        d1_buf.pop(0)
                    med = sorted(d1_buf)[len(d1_buf)//2]
                    if d1_f is None:
                        d1_f = med
                    else:
                        d1_f = ALPHA * med + (1.0 - ALPHA) * d1_f

                    # ===== 发串口 =====
                    send_frame(tx, D, d1_f)

                    # ===== 画面标记 =====
                    img.draw_rectangle([r[0], r[1], r[2], r[3]], color=(255, 0, 0))
                    img.draw_cross(best.cx(), best.cy(), color=(0, 255, 0))

                    # ===== 打印全套 =====
                    if n % PRINT_EVERY == 0:
                        # 顺便算一下"实测姿态"，方便和标定的 SIN_TH 对照
                        R = build_R(best.x_rotation(),
                                    best.y_rotation(),
                                    best.z_rotation())
                        uup = (R[0][1], R[1][1], R[2][1])

                        print("#%d tx=%+.4f ty=%+.4f tz=%+.4f uup=(%+.2f,%+.2f,%+.2f)" %
                              (n, tx, ty, tz, uup[0], uup[1], uup[2]))
                        print("     D=%.4f h=%+.4f hp=%+.4f | d1=%+.4f d1f=%+.4f | D2建议=%.4f fps=%.1f" %
                              (D, h, h_platform, d1_raw, d1_f,
                               SIZE + D3 - h, fps.fps()))

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