 # Import required modules
# 导入所需模块
import time, math, os, gc

from media.sensor import *
from media.display import *
from media.media import *

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
from machine import UART

# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()

last_id = None
stable_count = 0
STABLE_TH = 3   # 连续3帧确认

# AprilTag代码最多支持同时处理6种tag家族
tag_families = 0
tag_families |= image.TAG16H5
tag_families |= image.TAG25H7
tag_families |= image.TAG25H9
tag_families |= image.TAG36H10
tag_families |= image.TAG36H11
tag_families |= image.ARTOOLKIT


def family_name(tag):
    """
    获取tag的家族名称
    """
    family_dict = {
        image.TAG16H5: "TAG16H5",
        image.TAG25H7: "TAG25H7",
        image.TAG25H9: "TAG25H9",
        image.TAG36H10: "TAG36H10",
        image.TAG36H11: "TAG36H11",
        image.ARTOOLKIT: "ARTOOLKIT"
    }
    return family_dict.get(tag.family())


# 初始化摄像头
sensor = Sensor()
sensor.reset()
sensor.set_framesize(width=160, height=120)
sensor.set_pixformat(Sensor.RGB565)

# 初始化显示
Display.init(Display.ST7701, width=640, height=480, to_ide=True)
# Display.init(Display.VIRT, sensor.width(), sensor.height())

# 初始化媒体管理器
MediaManager.init()
sensor.run()

# FPS计时器
clock = time.clock()

DEBUG = True

# 主循环
while True:
    clock.tick()
    #uart.send(b"111\r\n")

    # 捕获图像
    img = sensor.snapshot()
    img.gaussian(1)

    # 查找AprilTags
    #for tags in img.find_apriltags(families=tag_families):
    # 查找AprilTags
    tags = img.find_apriltags(families=tag_families)

    if tags:
        # 选面积最大的
        tag = max(tags, key=lambda t: t.w() * t.h())

        #多帧检测
        if tag.id() == last_id:
            stable_count += 1
        else:
            stable_count = 0

        last_id = tag.id()

        # 没稳定直接跳过
        if stable_count < STABLE_TH:
            continue

        # 画框和中心点
        if DEBUG:
            img.draw_rectangle(tag.rect(), color=(255, 0, 0), thickness=4)
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0), thickness=2)

        # 打印信息
        rotation_deg = (180 * tag.rotation()) / math.pi
        print("Tag Family %s, Tag ID %d, rotation %f (degrees)" %
              (family_name(tag), tag.id(), rotation_deg))

        # 发送数据
        x, y, w, h = tag.rect()

        # 在框右上角显示ID
        img.draw_string(x + w - 40, y, "ID:%d" % tag.id(), color=(0,255,0), scale=2)
        pto_data = pto.get_apriltag_data(x, y, w, h, tag.id(), rotation_deg)

        uart.send(pto_data)
        print(pto_data)


    # 打印FPS
    fps = clock.fps()
    img.draw_string(0, 0, "FPS: %.2f" % fps, color=(255,255,0), scale=2)

    # 显示图像
    Display.show_image(img, x=120, y=120)
    #Display.show_image(img)


