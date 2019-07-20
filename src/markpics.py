#!/usr/bin/python2.7
import rospy
import os
import cv2 as cv
import numpy as np
from time import time
from std_msgs.msg import Int64, Float64, Header, String
from sensor_msgs.msg import CompressedImage
from constant import AnsiCode
from vision_lib import OutputTools, ImageTools, TransformTools

image = ImageTools(sub_sampling=0.3)
output = OutputTools(topic='/vision/topidos/')
transform = TransformTools()
seq = 1


def marking():
    if image.bgr is None:
        output.img_is_none()
        return
    image.renew_display()
    cv.rectangle(image.display, (102, 160), (188, 246), (0, 255, 0), 2)
    # 145 - (581/2) / (581/2) = -0.500860585197934 = x
    # 203 - (365/2) / (365/2) = 0.11232876712328767 = y
    # (-0.500860585197934,0.11232876712328767)
    cv.circle(image.display, (145, 203), 5, (0, 0, 255), 1)
    cv.circle(image.display, (145, 203), 10, (0, 0, 255), 1)
    cv.line(image.display, (130, 203), (160, 203), (0, 0, 255), 1)
    cv.line(image.display, (145, 188), (145, 218), (0, 0, 255), 1)
    output.log('Publishing', AnsiCode.GREEN)
    output.publish(image.display, 'bgr', 'firepoint')


if __name__ == '__main__':
    rospy.init_node('vision_stake', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber(image.topic('front'), CompressedImage, image.callback)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    while not rospy.is_shutdown():
        marking()
    rospy.spin()
    output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
