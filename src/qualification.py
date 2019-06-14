#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
from time import time
from std_msgs.msg import Int64, Float64, Header, String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from zeabus_utility.msg import VisionQualification
from zeabus_utility.srv import VisionSrvQualification
from operator import itemgetter
from constant import AnsiCode
from vision_lib import OutputTools, ImageTools, TransformTools

image = ImageTools(sub_sampling=0.3)
output = OutputTools(topic='/vision/qualification/')
transform = TransformTools()
seq = 1


class Log:
    def __init__(self):
        self.time = time()

    def reset_all(self):
        pass


def mission_callback(msg):
    """
        When call service it will run this 
        Returns:
            a group of process value from this program
    """
    if image.bgr is None:
        output.img_is_none()
        return message(state=-1)
    task = str(msg.task.data)
    request = str(msg.request.data)
    if task == 'qualification' and request == 'gate':
        return find_gate()
    elif task == 'qualification' and request == 'marker':
        return find_marker()


def message(state=0, cx1=0.0, cy1=0.0, cx2=0.0, cy2=0.0,
            name='qualification'):
    global seq
    if state < 0:
        return VisionQualification()
    msg = VisionQualification()
    header = Header()
    header.stamp = rospy.Time.now()
    header.seq = seq
    seq += 1
    msg.header = header
    msg.type = Int64(state)
    msg.name = String(name)
    msg.point1 = transform.to_point(cx1, cy1, image.display.shape[:2])
    msg.point2 = transform.to_point(cx2, cy2, image.display.shape[:2])
    print(msg)
    return msg


def get_mask():
    image.to_hsv()
    upper = np.array([72, 255, 255], dtype=np.uint8)
    lower = np.array([28, 0, 0], dtype=np.uint8)
    mask = cv.inRange(image.hsv, lower, upper)
    return mask


def get_obj(mask, align='vertical'):
    """
        Filter some noise
    """
    contours, _ = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    result = []
    for cnt in contours:
        if not is_pipe(cnt, area_ratio_expected=0.3, wh_ratio_expected=5,
                       align=align):
            continue
        (x, y), (w, h), angle = rect = cv.minAreaRect(cnt)
        w, h = max(w, h), min(w, h)
        if align == 'vertical':
            result.append([int(x), int(y), int(h), int(w), angle])
        elif align == 'horizontal':
            print(rect)
            result.append([int(x), int(y), int(w), int(h), angle])

    if align == 'vertical':
        result = sorted(result, key=itemgetter(3), reverse=True)
    elif align == 'horizontal':
        result = sorted(result, key=itemgetter(2), reverse=True)

    if align == 'horizontal':
        return result[:1]
    return result[:2]


def is_align(align, h, w):
    if align == "horizontal":
        if w <= h:
            return True
        return False
    elif align == "vertical":
        if h <= w:
            return True
        return False


def is_pipe(cnt, area_ratio_expected, wh_ratio_expected, align):
    x, y, w, h = cv.boundingRect(cnt)

    if is_align(align, h, w):
        return False

    (x, y), (w, h), angle = cv.minAreaRect(cnt)
    if not (angle >= -25 or angle <= -65):
        return False

    w, h = max(w, h), min(w, h)

    if w < 50:
        return False

    area_ratio_expected = 0.3
    area_cnt = cv.contourArea(cnt)
    area_box = w*h
    area_ratio = area_cnt / area_box
    if area_ratio < area_ratio_expected:
        return False

    wh_ratio_expected = 5
    wh_ratio = 1.0 * w / h

    if wh_ratio < wh_ratio_expected:
        return False

    return True


def find_gate():
    if image.bgr is None:
        output.img_is_none()
        return message(state=-1)

    image.renew_display()
    mask = get_mask()
    obj = mask
    # obj = image.bg_subtraction(mode='neg', bg_blur_size=85, fg_blur_size=11)

    output.publish(obj, 'gray', subtopic='obj')
    kernel_vertical = image.get_kernel(ksize=(1, 25))
    vertical = cv.erode(obj.copy(), kernel_vertical)
    kernel_box = image.get_kernel(ksize=(7, 7))
    vertical = cv.dilate(vertical.copy(), kernel_box)
    kernel_erode = image.get_kernel(ksize=(3, 11))
    vertical = cv.erode(vertical.copy(), kernel_erode)

    kernel_horizontal = image.get_kernel(ksize=(21, 1))
    horizontal = cv.erode(obj.copy(), kernel_horizontal)
    horizontal = cv.dilate(horizontal.copy(), kernel_box)
    kernel_erode = image.get_kernel(ksize=(11, 3))
    horizontal = cv.erode(horizontal.copy(), kernel_erode)

    output.publish(vertical, 'gray', subtopic='mask/vertical')
    output.publish(horizontal, 'gray', subtopic='mask/horizontal')

    vertical_pipe = get_obj(vertical, align='vertical')
    horizontal_pipe = get_obj(horizontal, align='horizontal')

    vertical_cx1 = []
    vertical_cx2 = []
    vertical_cy1 = []
    vertical_cy2 = []
    print(vertical_pipe)
    for res in vertical_pipe:
        x, y, w, h, angle = res
        cv.rectangle(image.display, (int(x - w / 2.), int(y - h / 2.)),
                     (int(x + w / 2.), int(y + h / 2.)), (108, 105, 255), 2)
        vertical_cx1.append((x - w / 2.))
        vertical_cx2.append((x + w / 2.))
        vertical_cy1.append((y - h / 2.))
        vertical_cy2.append((y + h / 2.))

    horizontal_cx1 = []
    horizontal_cx2 = []
    horizontal_cy1 = []
    horizontal_cy2 = []
    for res in horizontal_pipe:
        x, y, w, h, angle = res
        cv.rectangle(image.display, (int(x - w / 2.), int(y - h / 2.)),
                     (int(x + w / 2.), int(y + h / 2.)), (108, 105, 255), 2)
        horizontal_cx1.append((x - w / 2.))
        horizontal_cx2.append((x + w / 2.))
        horizontal_cy1.append((y - h / 2.))
        horizontal_cy2.append((y + h / 2.))

    himg, wimg = image.bgr.shape[:2]
    num_vertical = len(vertical_pipe)
    num_horizontal = len(horizontal_pipe)
    print('v,h', num_vertical, num_horizontal)
    # cx1 = min(vertical_cx2)
    # cx2 = max(vertical_cx1)
    # cy1 = max(vertical_cy1)
    # cy2 = min(vertical_cy2)

    # cx1, cx2 = min(cx1, cx2), max(cx1, cx2)
    # cy1, cy2 = min(cy1, cy2), max(cy1, cy2)

    # cx1, cx2 = max(cx1, 0), min(cx2, wimg)
    # cy1, cy2 = max(cy1, 0), min(cy2, himg)
    if num_vertical == 0 and num_horizontal == 0:
        output.log("NOT FOUND", AnsiCode.RED)
        output.publish(image.display, 'bgr', subtopic='display')
        output.publish(vertical, 'gray', subtopic='mask/vertical')
        output.publish(obj, 'gray', subtopic='mask')
        return message()
    elif num_vertical == 1 and num_horizontal == 0:
        output.log("FOUND ONE V", AnsiCode.YELLOW)
        state = 1
        cx1 = vertical_cx1[0]
        cy1 = vertical_cy1[0]
        cx2 = wimg
        cy2 = vertical_cy2[0]
    elif num_vertical == 1 and num_horizontal == 1:
        output.log("FOUNG ONE V AND ONE H", AnsiCode.YELLOW)
        if vertical_cx1[0] < vertical_cx2[0] and vertical_cx1[0] < horizontal_cx1[0] :
            state = 2 
            cx1 = vertical_cx2[0]
            cy1 = horizontal_cy2[0]
            cx2 = wimg
            cy2 = vertical_cy2[0]
        else:
            state = 4
            cx1 = 0
            cy1 = horizontal_cy2[0]
            cx2 = vertical_cx1[0]
            cy2 = vertical_cy2[0]
    elif num_vertical == 0 and num_horizontal == 1:
        output.log("FOUNG ONE H", AnsiCode.YELLOW)
        state = 3
        cx1 = horizontal_cx1[0]
        cy1 = horizontal_cy2[0]
        cx2 = horizontal_cx2[0]
        cy2 = himg
    elif num_vertical == 2:
        output.log("FOUND", AnsiCode.GREEN)
        state = 5
        cx1 = min(vertical_cx2[0],vertical_cx2[1])
        cy1 = horizontal_cy2[0]
        cx2 = max(vertical_cx1[0], vertical_cx1[1])
        cy2 = max(vertical_cy2[0], vertical_cy2[1])
    elif num_vertical == 2 and num_horizontal == 0:
        output.log("FOUND GATE", AnsiCode.GREEN)
        state = 5
        cx1 = vertical_cx2[0]
        cy1 = 0
        cx2 = vertical_cx1[1]
        cy2 = himg


    cx1, cx2 = max(cx1, 0), min(cx2, wimg)
    cy1, cy2 = max(cy1, 0), min(cy2, himg)

    cv.rectangle(image.display, (int(cx1), int(cy1)),
                 (int(cx2), int(cy2)), (0, 255, 0), 3)
    cv.circle(image.display, (int((cx1+cx2)/2), int((cy1+cy2)/2)),
              3, (0, 255, 255), -1)

    # area = 1.0*abs(cx2-cx1)*abs(cy2-cy1)/(himg*wimg)
    output.publish(image.display, 'bgr', subtopic='display')
    output.publish(vertical, 'gray', subtopic='mask/vertical')
    output.publish(obj, 'gray', subtopic='mask')
    # return message(state=state)
    return message(state=state, cx1=cx1, cy1=cy1, cx2=cx2,
                   cy2=cy2)


def find_marker():
    mask = get_mask()
    obj = get_obj(mask)
    return message()


if __name__ == '__main__':
    rospy.init_node('vision_qualification', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber(image.topic('front'), CompressedImage, image.callback)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    rospy.Service('vision/qualification',
                  VisionSrvQualification(), mission_callback)
    output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
