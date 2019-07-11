#!/usr/bin/python2.7
import rospy
import os
import cv2 as cv
import numpy as np
from time import time
from std_msgs.msg import Int64, Float64, Header, String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from zeabus_utility.msg import VisionBox
from zeabus_utility.srv import VisionSrvExposed
from operator import itemgetter
from constant import AnsiCode
from vision_lib import OutputTools, ImageTools, TransformTools

image = ImageTools(sub_sampling=0.5)
output = OutputTools(topic='/vision/stake/')
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


def message(state=0, pt1=(0,0), pt2=(0,0), pt3=(0,0), pt4=(0,0), area=0.0,
            name='coffin'):
    global seq
    if state < 0:
        return VisionBox()
    msg = VisionBox()
    header = Header()
    header.stamp = rospy.Time.now()
    header.seq = seq
    seq += 1
    msg.header = header
    shape = image.display.shape[:2]
    msg.point_1 = transform.convert_to_point(pt1, shape)
    msg.point_2 = transform.convert_to_point(pt2, shape)
    msg.point_3 = transform.convert_to_point(pt3, shape)
    msg.point_4 = transform.convert_to_point(pt3, shape)
    msg.area = transform.convert(area,shape[0]*shape[1])
    msg.state = state
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
    
    return 

if __name__ == '__main__':
    rospy.init_node('vision_exposed', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber(image.topic('front'), CompressedImage, image.callback)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    # cv.namedWindow('frame')
    # cv.createTrackbar('lower', 'frame', 0, 255, nothing)
    c = 0
    while not rospy.is_shutdown():
        a = time()
        find_coffin(c)
        c += 1
        print('time',time()-a)
        # rospy.sleep(0.1)
    # rospy.Service('vision/stake',
    #               VisionSrvStake(), mission_callback)
    # output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    # output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
