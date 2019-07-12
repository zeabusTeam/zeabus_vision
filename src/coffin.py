#!/usr/bin/python2.7
import rospy
import os
import cv2 as cv
import numpy as np
from time import time
from std_msgs.msg import Int64, Float64, Header, String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from zeabus_utility.msg import VisionBox, VisionCoffins
from zeabus_utility.srv import VisionSrvExposed
from operator import itemgetter
from constant import AnsiCode
from vision_lib import OutputTools, ImageTools, TransformTools

image = ImageTools(sub_sampling=0.5)
output = OutputTools(topic='/vision/exposed')
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
    if task == 'exposed' and request == 'coffin':
        return find_coffin()


def to_box(state=0, box=0, area=0.0):
    shape = image.display.shape[:2]
    print('b', box)
    sort = sorted(box, key=lambda x: x[0])
    bottom = sort[:2]
    bottom = sorted(bottom, key=lambda x: x[1])
    pt4 = bottom[0]
    pt2 = bottom[1]
    top = sort[2:]
    top = sorted(top, key=lambda x: x[1])
    pt3 = top[0]
    pt1 = top[1]
    box = VisionBox()
    for i in range(1, 5):
        print('pt'+str(i), tuple(eval('pt'+str(i))))
        cv.putText(image.display, str(i), tuple(eval('pt'+str(i))),
                   cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2, cv.LINE_AA)
    box.point_1 = transform.convert_to_point(pt1, shape)
    box.point_2 = transform.convert_to_point(pt2, shape)
    box.point_3 = transform.convert_to_point(pt3, shape)
    box.point_4 = transform.convert_to_point(pt4, shape)
    box.area = transform.convert(area, shape[0]*shape[1])
    return box


def message(box1=0, box2=0, state=0, name='coffin'):
    global seq
    if state <= 0:
        return VisionCoffins()
    msg = VisionCoffins()
    header = Header()
    header.stamp = rospy.Time.now()
    header.seq = seq
    seq += 1
    msg.header = header
    msg.state = state
    msg.name = name
    if box2 == 0:
        print('a1', box1)
        msg.data = [to_box(box=box1), VisionBox()]
    else:
        msg.data = [to_box(box=box1), to_box(box=box2)]
    output.publish(image.display, 'bgr', '/display')
    print(msg)
    return msg


def get_mask():
    image.to_hsv()
    upper = np.array([72, 255, 255], dtype=np.uint8)
    lower = np.array([28, 0, 0], dtype=np.uint8)
    mask = cv.inRange(image.hsv, lower, upper)
    return mask


def find_coffin(c=0):
    if image.bgr is None:
        output.img_is_none()
        return message(state=-1)
    image.renew_display()
    mask = get_mask()
    output.publish(mask, 'gray', '/mask')
    contours = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    res = []
    for cnt in contours:
        if cv.contourArea(cnt) < 10000:
            continue
        res.append(cnt)
    cnt = sorted(res, reverse=True)
    rect = cv.minAreaRect(cnt[0])
    box1 = cv.boxPoints(rect)
    box1 = np.int64(box1)
    cv.drawContours(image.display, [box1], 0, (0, 0, 255), 2)
    if len(cnt) > 1:
        rect = cv.minAreaRect(cnt[1])
        box2 = cv.boxPoints(rect)
        box2 = np.int64(box2)
        cv.drawContours(image.display, [box2], 0, (0, 0, 255), 2)
    print('len(cnt)', len(cnt))
    if len(cnt) == 0:
        return message(state=0)
    elif len(cnt) == 1:
        return message(box1=box1, state=1)
    elif len(cnt) == 2:
        return message(box1=box1, box2=box2, state=1)
    return message(state=-2)


if __name__ == '__main__':
    rospy.init_node('vision_exposed', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber(image.topic('bottom'), CompressedImage,
                     image.callback, queue_size=1)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    # cv.namedWindow('frame')
    # cv.createTrackbar('lower', 'frame', 0, 255, nothing)
    c = 0
    while not rospy.is_shutdown():
        a = time()
        find_coffin(c)
        c += 1
        print('time', time()-a)
        # rospy.sleep(0.1)
    # rospy.Service('vision/stake',
    #               VisionSrvStake(), mission_callback)
    # output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    # output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
