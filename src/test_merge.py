#!/usr/bin/python2.7
"""
    File name: gate (qualification)
    Author: AyumiizZ
    Date created: 2019/3/5
    Python Version: 2.7
"""

import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import CompressedImage, Image
from zeabus_utility.msg import VisionObject, VisionSAUVC
from vision_lib import OutputTools, AnsiCode

output = OutputTools('/all_task')
result_pub = output.create_publish(datatype=VisionSAUVC, subtopic='/result')

def gate_sub():
    pass

def flare_sub():
    pass

def drum_sub():
    pass

def golf_sub():
    pass


if __name__ == '__main__':
    rospy.init_node('vision_gate', anonymous=False)
    rospy.Subscriber(image.topic('front'), CompressedImage, image.callback)
    output.log("INIT NODE gate", AnsiCode.GREEN)
    rospy.spin()
    output.log("END PROGRAM", AnsiCode.YELLOW_HL+AnsiCode.RED)