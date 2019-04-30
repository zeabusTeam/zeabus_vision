#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
from time import time
from std_msgs.msg import Int64, Float64
from sensor_msgs.msg import CompressedImage, Image
from zeabus_utility.msg import VisionQualification
from zeabus_utility.srv import VisionSrvQualification

# from vision_lib import *


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
    # print_result('mission_callback', color_text.CYAN)

    # task = msg.task.data
    msg = message()
    return msg

    # print('task:', str(task))
    # if task == 'casino_gate':
    #     return find_gate()


def message(state=0, pos=0, cx1=0.0, cy1=0.0, cx2=0.0, cy2=0.0, area=0.0):
    msg = VisionQualification()
    msg.state = Int64(state)
    msg.pos = Int64(pos)
    msg.cx1 = Float64(cx1)
    msg.cy1 = Float64(cy1)
    msg.cx2 = Float64(cx2)
    msg.cy2 = Float64(cy2)
    msg.area = Float64(area)
    return msg


if __name__ == '__main__':
    rospy.init_node('vision_qualification', anonymous=False)
    # print_result("INIT NODE", color_text.GREEN)
    # image_topic = "/stereo/right/image_color/compressed"
    # image_topic = get_topic("front", world)
    # rospy.Subscriber(image_topic, CompressedImage, image_callback)
    # print_result("INIT SUBSCRIBER", color_text.GREEN)
    rospy.Service('vision/service/qualification',
                  VisionSrvQualification(), mission_callback)
    # print_result("INIT SERVICE", color_text.GREEN)
    rospy.spin()
    # print_result("END PROGRAM", color_text.YELLOW_HL+color_text.RED)
