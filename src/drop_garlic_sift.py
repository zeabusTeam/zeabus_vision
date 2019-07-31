#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
import math
from time import time
from sensor_msgs.msg import CompressedImage
from zeabus_utility.msg import VisionBox
from zeabus_utility.srv import VisionSrvDropGarlic
from constant import AnsiCode
from vision_lib import OutputTools, ImageTools, TransformTools
from geometry_msgs.msg import Point

image = ImageTools(sub_sampling=0.3)
output = OutputTools(topic='/vision/drop_garlic/')
bat_detector = Detector(picture_name='bat.png')
wolf_detector = Detector(picture_name='wolf.png')
transform = TransformTools()


def mission_callback(msg):
    """
        When call service it will run this 
        Returns:
            a group of process value from this program
    """
    if image.bgr is None:
        output.img_is_none()
        return None
    task = str(msg.task.data)
    request = str(msg.request.data)
    print request
    if task == 'drop_garlic':
        if request == 'search':
            return find_drop_garlic(func='search')
        elif request == 'open':
            return find_open()
        elif request == 'drop':
            return find_drop()


def message(wimg, himg, state, cx1=0.0, cy1=0.0, cx2=0.0, cy2=0.0, cx3=0.0,
            cy3=0.0, cx4=0.0, cy4=0.0, area=0.0):
    msg = VisionBox()
    msg.state = state
    msg.point_1 = (100*transform.convert(cx1, wimg), -
                   100.0*transform.convert(cy1, himg))
    msg.point_2 = (100*transform.convert(cx2, wimg), -
                   100.0*transform.convert(cy2, himg))
    msg.point_3 = (100*transform.convert(cx3, wimg), -
                   100.0*transform.convert(cy3, himg))
    msg.point_4 = (100*transform.convert(cx4, wimg), -
                   100.0*transform.convert(cy4, himg))
    msg.area = area*100
    print msg
    return msg


def get_mask():
    image.renew_display()

    max_iter = 5

    mask = image.bg_subtraction_kmean(
        image.display, bg_k=1, fg_k=3, mode='pos', max_iter=max_iter)

    output.publish(mask, 'gray', '/mask')

    image.to_gray()
    output.publish(image.gray, 'gray', '/gray')

    return mask


def get_cx(box, cnt, mode):
    min_box = 1000
    max_box = 0
    for a in range(4):
        sum_box = sum(box[a])
        if sum_box < min_box:
            min_box = sum_box
            cx4 = box[a][0]
            cy4 = box[a][1]
        if sum_box > max_box:
            max_box = sum_box
            cx2 = box[a][0]
            cy2 = box[a][1]
    if mode == 'find_mission' and (min_box == 0 or max_box == 1000):
        return 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    min_x = 1000
    max_x = 0
    for c in range(4):
        if sum(box[c]) != cx2+cy2 and sum(box[c]) != cx4+cy4:
            if box[c][0] < min_x:
                min_x = box[c][0]
                cx1 = box[c][0]
                cy1 = box[c][1]
            if box[c][0] > max_x:
                max_x = box[c][0]
                cx3 = box[c][0]
                cy3 = box[c][1]
    if (max_x == 0 or min_x == 1000):
        return 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    if (mode == 'find_mission' and (cx1 <= 40 or cx2 >= 560 or cx3 >= 560 or cx4 <= 40 or cy1 >= 344 or cy2 >= 344 or cy3 <= 20 or cy4 <= 20)) or (mode == 'find_slice' and (cx1 <= 10 or cx2 >= 100 or cx3 >= 300 or cx4 <= 10 or cy1 >= 100 or cy2 >= 100 or cy3 <= 10 or cy4 <= 10)):
        M = cv.moments(cnt)
        if M["m00"] != 0:
            cx1 = int(M["m10"]/M["m00"])
            cy1 = int(M["m01"]/M["m00"])
        else:
            cx1 = 0
            cy1 = 0
        return 2, cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4
    return 1, cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4


def find_drop_garlic(func):
    image.renew_display()
    image.to_gray()

    blur = cv.GaussianBlur(image.gray, (5, 5), 0)
    himg, wimg = blur.shape

    blur = blur[20:-20, :]

    blur = cv.GaussianBlur(image.gray, (5, 5), 0)
    ret1, th1 = cv.threshold(blur, blur.max()*0.7, 255, cv.THRESH_BINARY)

    edges = cv.Canny(blur, ret1*9/10, ret1)
    output.publish(edges, 'gray', '/canny_be')

    output.publish(th1, 'gray', '/th1')

    edges = cv.GaussianBlur(edges, (5, 5), 0)
    kernel = np.ones((1, 21), np.uint8)
    edges = cv.dilate(edges, kernel)
    kernel = np.ones((11, 1), np.uint8)
    edges = cv.dilate(edges, kernel)
    output.publish(edges, 'gray', '/test')

    mask = get_mask()
    cnt = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]

    if len(cnt) > 0:
        cnt = max(cnt, key=cv.contourArea)
        rect = cv.minAreaRect(cnt)
        w_cnt = rect[1][0]
        h_cnt = rect[1][1]
        box = cv.boxPoints(rect)
        box = np.int0(box)
        approx = cv.approxPolyDP(cnt, 0.025*cv.arcLength(cnt, True), True)
        print len(approx)
        area = cv.contourArea(cnt)
        print area
        print area/(w_cnt*h_cnt)
        if area > 3000 and area/(w_cnt*h_cnt) > 0.6:
            state, cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4 = get_cx(
                box, cnt, mode='find_mission')

            if (state == 1 or func == 'drop') and len(approx) <= 10:
                cv.drawContours(image.display, [box], 0, (255, 255, 255), 2)
                cv.circle(image.display, (cx4, cy4), 3, (255, 0, 0), -1)
                cv.circle(image.display, (cx2, cy2), 3, (0, 255, 0), -1)
                cv.circle(image.display, (cx3, cy3), 3, (0, 0, 255), -1)
                cv.circle(image.display, (cx1, cy1), 3, (0, 0, 0), -1)
            elif state == 2 and len(approx) <= 15:
                cv.circle(image.display, (cx1, cy1), 3, (255, 255, 255), -1)
                cx2 = 0.0
                cx3 = 0.0
                cx4 = 0.0
                cy2 = 0.0
                cy3 = 0.0
                cy4 = 0.0
            output.publish(image.display, 'bgr', '/cnt')
            if func == 'search':
                return message(wimg, himg, state, cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4, area/(himg*wimg))
            else:
                return wimg, himg, cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4, edges

    if func == 'search':
        return message(wimg, himg, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    else:
        return wimg, himg, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0


def find_drop():
    


if __name__ == '__main__':
    rospy.init_node('vision_drop_garlic', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber('/vision/bottom/image_raw/compressed',
                     CompressedImage, image.callback)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    rospy.Service('/vision/drop_garlic',
                  VisionSrvDropGarlic(), mission_callback)
    output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
