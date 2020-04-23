#!/usr/bin/python2.7
"""
    File name: gate
    Author: AyumiizZ
    Date created: 2020/02/02
    Python Version: 2.7
"""

import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import CompressedImage, Image
from zeabus_utility.msg import VisionObject
from vision_lib import OutputTools, AnsiCode, ImageTools, TransformTools

from operator import itemgetter
from time import time


image = ImageTools(sub_sampling=0.3)
output = OutputTools(topic='/vision/gate')
image_publisher = output.create_publish(datatype=Image, subtopic='/result')
transform = TransformTools()

def message(state=0, pos=0, cx1=0.0, cy1=0.0, cx2=0.0, cy2=0.0, area=0.0):
    msg = VisionObject()
    msg.label = 'gate'
    msg.state = state
    msg.area = area
    msg.point_1 = [cx1, cy1]
    msg.point_2 = [cx2, cy1]
    msg.point_3 = [cx2, cy2]
    msg.point_4 = [cx1, cy2]
    return msg


def is_verticle_pipe(cnt, percent, rect):
    """
        Information
        Pipe    width = 100
                height = 4
    """

    x, y, w, h = cv.boundingRect(cnt)
    if h <= w:
        return False

    (x, y), (w, h), angle = rect
    if not (angle >= -25 or angle <= -65):
        return False

    area_cnt = cv.contourArea(cnt)
    area_box = w * h
    w, h = max(w, h), min(w, h)
    if area_box <= 50 or area_cnt <= 300 or w < 50:
        return False

    area_ratio_expected = 0.3
    area_ratio = area_cnt / area_box
    if area_ratio < area_ratio_expected:
        return False

    wh_ratio_expected = (100/4.)/2

    wh_ratio = 1.0 * w / h
    if wh_ratio < wh_ratio_expected * percent:
        return False

    return True


def find_pipe(binary):
    _, contours, _ = cv.findContours(
        binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    number_of_object = 2
    percent_pipe = 0.6
    result = []
    pipe = cv.cvtColor(binary, cv.COLOR_GRAY2BGR)

    for cnt in contours:
        (x, y), (w, h), angle = rect = cv.minAreaRect(cnt)
        w, h = max(w, h), min(w, h)
        if not is_verticle_pipe(cnt, percent_pipe, rect):
            continue

        box = cv.boxPoints(rect)
        box = np.int64(box)
        cv.drawContours(pipe, [box], 0, (0, 255, 255), 2)
        result.append([int(x), int(y), int(h), int(w), angle])
    output.publish_image(pipe, 'bgr', subtopic='/mask/pipe')
    result = sorted(result, key=itemgetter(3), reverse=True)

    if len(result) < number_of_object:
        return result, len(result)
    else:
        closest_pair = []
        min_dist = 2000
        for i in range(len(result)):
            for j in range(i+1, len(result)):
                dist_y = abs(result[j][1] - result[i][1])
                dist_x = abs(result[j][0] - result[i][0])
                if dist_x >= 50 and dist_y < min_dist:
                    min_dist = dist_y
                    closest_pair = [result[i], result[j]]
        if closest_pair == []:
            return result[:1], 1
        else:
            return closest_pair, 2


def find_gate():
    if image.bgr is None:
        output.img_is_none()
        return message(state=-1)

    image.renew_display()
    image.to_gray()
    obj = image.bg_subtraction(mode='neg', fg_blur_size=3)

    kernel_box = image.get_kernel(ksize=(7, 7))

    kernel_vertical = image.get_kernel(ksize=(1, 25))
    vertical = cv.erode(obj.copy(), kernel_vertical)
    vertical = cv.dilate(vertical.copy(), kernel_box)
    kernel_erode = image.get_kernel(ksize=(3, 11))
    vertical = cv.erode(vertical.copy(), kernel_erode)

    vertical_pipe, no_pipe_v = find_pipe(vertical)

    vertical_cx1 = []
    vertical_cx2 = []
    vertical_cy1 = []
    vertical_cy2 = []
    for res in vertical_pipe:
        x, y, w, h, angle = res
        cv.rectangle(image.display, (int(x - w / 2.), int(y - h / 2.)),
                     (int(x + w / 2.), int(y + h / 2.)), (108, 105, 255), 2)
        vertical_cx1.append((x - w / 2.))
        vertical_cx2.append((x + w / 2.))
        vertical_cy1.append((y - h / 2.))
        vertical_cy2.append((y + h / 2.))

    himg, wimg = obj.shape[:2]
    mode = no_pipe_v

    if mode == 0:
        output.log("NOT FOUND", AnsiCode.RED)
        output.publish_image(image.display, 'bgr', subtopic='/display')
        output.publish_image(vertical, 'gray', subtopic='/mask/vertical')
        output.publish_image(obj, 'gray', subtopic='/mask')
        return message()
    elif mode == 1:
        output.log("FOUNG ONE POLE", AnsiCode.YELLOW)
    elif mode == 2:
        output.log("FOUND", AnsiCode.GREEN)

    cx1 = min(vertical_cx2)
    cx2 = max(vertical_cx1)
    cy1 = max(vertical_cy1)
    cy2 = min(vertical_cy2)

    cx1, cx2 = min(cx1, cx2), max(cx1, cx2)
    cy1, cy2 = min(cy1, cy2), max(cy1, cy2)

    cx1, cx2 = max(cx1, 0), min(cx2, wimg)
    cy1, cy2 = max(cy1, 0), min(cy2, himg)

    cv.rectangle(image.display, (int(cx1), int(cy1)),
                 (int(cx2), int(cy2)), (0, 255, 0), 3)
    cv.circle(image.display, (int((cx1+cx2)/2), int((cy1+cy2)/2)),
              3, (0, 255, 255), -1)

    area = 1.0*abs(cx2-cx1)*abs(cy2-cy1)/(himg*wimg)
    pos = -7411
    cx1 = transform.convert(cx1, wimg)
    cx1 = transform.convert(-1*cy1, himg)
    cx1 = transform.convert(cx2, wimg)
    cx1 = transform.convert(-1*cy2, himg)
    output.publish_image(image.display, 'bgr', subtopic='/display')
    output.publish_image(vertical, 'gray', subtopic='/mask/vertical')
    output.publish_image(obj, 'gray', subtopic='/mask')
    return message(state=mode, cx1=cx1, cy1=cy1, cx2=cx2, cy2=cy2, pos=pos, area=area)


if __name__ == '__main__':
    rospy.init_node('vision_gate', anonymous=False)
    output.log("INIT NODE GATE", AnsiCode.GREEN)
    rospy.Subscriber(image.topic('front'), CompressedImage, image.callback)
    data_publisher = output.create_publish(
        datatype=VisionObject, subtopic='/data')
    while not rospy.is_shutdown():
        gate_msg = find_gate()
        print(gate_msg)
        data_publisher.publish(gate_msg)
        rospy.sleep(0.05)
        output.end_loop = True
    rospy.spin()
    output.log("END PROGRAM", AnsiCode.YELLOW_HL+AnsiCode.RED)
