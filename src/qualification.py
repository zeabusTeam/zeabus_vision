#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
from time import time
from std_msgs.msg import Int64, Float64
from sensor_msgs.msg import CompressedImage, Image
from zeabus_utility.msg import VisionQualification
from zeabus_utility.srv import VisionSrvQualification
from constant import AnsiCode
from vision_lib import OutputTools, Image

image = Image()
output = OutputTools()


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


def get_mask():
    image.to_hsv()
    upper = np.array([72, 255, 255], dtype=np.uint8)
    lower = np.array([28, 0, 0], dtype=np.uint8)
    mask = cv.inRange(image.hsv, lower, upper)
    return mask


def get_obj(mask):
    """
        Filter some noise
    """
    return mask


def find_gate():
    mask = get_mask()
    obj = get_obj(mask)
    return message()


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
