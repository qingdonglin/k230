# AprilTags Example
#
# This example shows the power of the CanMV Cam to detect April Tags.
# 我在测试提交
#
# 【示例功能】CanMV K230 摄像头检测 AprilTag 标签，并输出标签的位姿
#            （x/y/z 平移 + x/y/z 旋转），用于测距、定位、视觉对准等场景。

# ==================== 导入模块 ====================
import time, math, os, gc, sys
# time：计时用，time.clock() 创建帧率统计对象
# math：数学运算，弧度转角度要用到 math.pi
# os  ：系统接口，os.exitpoint() 用来响应 IDE 的"停止"操作
# gc  ：垃圾回收，循环内手动回收可减少内存碎片
# sys ：系统相关（本示例未直接用到）

from media.sensor import *    # 摄像头 Sensor 类、图像格式常量（GRAYSCALE 等）
from media.display import *   # 显示输出接口（VIRT / LT9611 / ST7701 等）
from media.media import *     # 媒体资源管理（本例未显式调用，随框架一起导入）

# Note! Unlike find_qrcodes the find_apriltags method does not need lens correction on the image to work.
# 【说明1】与 find_qrcodes 不同，find_apriltags 不需要先对图像做镜头畸变校正就能识别。

# What's the difference between tag families? Well, for example, the TAG16H5 family is effectively
# a 4x4 square tag. So, this means it can be seen at a longer distance than a TAG36H11 tag which
# is a 6x6 square tag. However, the lower H value (H5 versus H11) means that the false positve
# rate for the 4x4 tag is much, much, much, higher than the 6x6 tag. So, unless you have a
# reason to use the other tags families just use TAG36H11 which is the default family.
# 【说明2】标签家族（tag family）的区别：TAG16H5 本质是 4x4 方块，识别距离比 6x6 的 TAG36H11 更远；
#         但 H 值越小（H5 < H11）误识别率越高。所以除非有特殊需求，默认都用 TAG36H11。

# The AprilTags library outputs the pose information for tags. This is the x/y/z translation and
# x/y/z rotation. The x/y/z rotation is in radians and can be converted to degrees. As for
# translation the units are dimensionless and you must apply a conversion function.
# 【说明3】AprilTags 库输出标签的位姿：x/y/z 平移 + x/y/z 旋转。
#         旋转量单位是弧度，可换算成角度；平移量无量纲，必须乘换算系数才是真实距离。

# f_x is the x focal length of the camera. It should be equal to the lens focal length in mm
# divided by the x sensor size in mm times the number of pixels in the image.
# The below values are for the OV7725 camera with a 2.8 mm lens.
# 【说明4】f_x：x 方向焦距（像素）= 镜头焦距(mm) / 传感器 x 尺寸(mm) × 图像宽度(像素)。
#         下面的数值对应 OV7725 摄像头 + 2.8mm 镜头。

# f_y is the y focal length of the camera. It should be equal to the lens focal length in mm
# divided by the y sensor size in mm times the number of pixels in the image.
# The below values are for the OV7725 camera with a 2.8 mm lens.
# 【说明5】f_y：y 方向焦距（像素）= 镜头焦距(mm) / 传感器 y 尺寸(mm) × 图像高度(像素)。
#         下面的数值同样对应 OV7725 + 2.8mm 镜头。

# c_x is the image x center position in pixels.
# c_y is the image y center position in pixels.
# 【说明6】c_x / c_y：图像主点（光心）的 x / y 像素坐标。

DETECT_WIDTH  = 640
DETECT_HEIGHT = 480

# === 标定内参 (仅对 640x480 有效) ===
f_x = 790.62
f_y = 790.41
c_x = 320.36
c_y = 247.18
def degrees(radians):
    """弧度 → 角度：角度 = 弧度 × 180 / π"""
    return (180 * radians) / math.pi

sensor = None  # 提前声明为 None，finally 中用它判断摄像头是否创建成功

try:
    # 创建 Sensor 对象：按给定宽高构造，随后再精细设置各通道参数
    sensor = Sensor(width = DETECT_WIDTH, height = DETECT_HEIGHT)
    # 复位摄像头（内部完成初始化，必须最先调用）
    sensor.reset()
    # 水平镜像（默认关闭，需要时取消注释）
    # sensor.set_hmirror(False)
    # 垂直翻转（默认关闭，需要时取消注释）
    # sensor.set_vflip(False)
    # 设置 chn0（通道0）输出尺寸，crop=True 表示裁剪输出而非缩放
    sensor.set_framesize(width = DETECT_WIDTH, height = DETECT_HEIGHT)
    # 设置 chn0 输出格式为灰度图：AprilTag 检测只需灰度信息，更省带宽和内存
    sensor.set_pixformat(Sensor.GRAYSCALE)

    # 【可选】HDMI 输出，分辨率 VGA(640x480)
    # Display.init(Display.LT9611, width = 640, height = 480, to_ide = True)

    # 【可选】HDMI 输出，分辨率 1080P
    # Display.init(Display.LT9611, width = 1920, height = 1080, to_ide = True)

    # 【可选】MIPI LCD 屏输出
    # Display.init(Display.ST7701, to_ide = True)

    # 【当前使用】Display.VIRT：虚拟显示，画面直接送 IDE 预览窗口
    # to_ide=True 表示同步到 CanMV IDE，fps=60 为显示通道目标帧率
    Display.init(Display.VIRT, width = DETECT_WIDTH, height = DETECT_HEIGHT, fps = 60, to_ide = True)



    # 启动摄像头，开始采集图像
    sensor.run()

    fps = time.clock()  # 创建帧率统计对象，用 fps.tick() / fps.fps() 统计

    while True:
        fps.tick()  # 标记本帧开始，用于帧率统计

        # 检查是否需要退出：IDE 点"停止"或收到 Ctrl+C 时，这里会抛出异常
        os.exitpoint()
        img = sensor.snapshot()  # 抓取一帧图像
        # 在图像中查找 AprilTag：family 未指定时默认 TAG36H11
        # fx/fy/cx/cy 为相机内参，用于由标签像素位置反解位姿（平移/旋转）
        for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y): # defaults to TAG36H11
            img.draw_rectangle([v for v in tag.rect()], color=(255, 0, 0))  # 红色矩形框住标签（tag.rect() 返回 4 个角点）
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))           # 绿色十字标出标签中心

            size = 0.048                        # 标签真实边长(米)：请按实际打印尺寸修改

            # 平移 → 米（tag widths × 边长）
            # 库输出的平移量是"标签边长的倍数"，乘真实边长即得到米
            tx = tag.x_translation() * size     # 左右偏移 (m)
            ty = tag.y_translation() * size     # 上下偏移 (m)
            tz = tag.z_translation() * size     # 前后距离 (m)

            # 旋转 → 度（本来就是真实角度，不用乘）
            rx = degrees(tag.x_rotation())      # 绕 x 轴旋转角(°)
            ry = degrees(tag.y_rotation())      # 绕 y 轴旋转角(°)
            rz = degrees(tag.z_rotation())      # 绕 z 轴旋转角(°)

            # 打印标签 ID、平移(m) 与旋转角(°)
            print("ID:%d  Tx:%.3f Ty:%.3f Tz:%.3f  Rx:%.2f Ry:%.2f Rz:%.2f"
                  % (tag.id(), tx, ty, tz, rx, ry, rz))

        # 把画好标记的这一帧显示出来（VIRT 模式即推送到 IDE 预览窗口）
        Display.show_image(img)
        gc.collect()  # 手动回收内存，避免长时间运行产生内存碎片

        print(fps.fps())  # 打印当前帧率
except KeyboardInterrupt as e:
    print(f"user stop")        # 用户按 Ctrl+C 或 IDE 点"停止"时进入此分支
except BaseException as e:
    print(f"Exception '{e}'")  # 其它异常：打印出来便于排查问题
finally:
    # 无论正常结束还是异常，都要释放资源
    # sensor stop run
    if isinstance(sensor, Sensor):  # 摄像头创建成功才需要停止
        sensor.stop()
    # deinit display
    Display.deinit()                # 关闭显示输出

    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)  # 重新打开退出点，允许进入休眠以便被中断
    time.sleep_ms(100)                       # 延时 100ms，等资源彻底释放