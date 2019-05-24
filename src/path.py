#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
from time import time
from std_msgs.msg import Int64, Float64
from sensor_msgs.msg import CompressedImage
from zeabus_utility.msg import VisionPath
from zeabus_utility.srv import VisionSrvPath
from constant import AnsiCode
from vision_lib import OutputTools, ImageTools

image = ImageTools()
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
    if task == 'path' and request != None :
        return find_path() 


def message(cx1=0.0, cy1=0.0, cx2=0.0, cy2=0.0, cx3=0.0, cy3=0.0):
    msg.cx1 = Float64(cx1)
    msg.cy1 = Float64(cy1)
    msg.cx2 = Float64(cx2)
    msg.cy2 = Float64(cy2)
    msg.cx3 = Float64(cx3)
    msg.cy3 = Float64(cy3)
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


def find_path():
    mask = get_mask()
    # obj = get_obj(mask)
    return message()


if __name__ == '__main__':
    rospy.init_node('vision_path', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber(image.topic('bottom'), CompressedImage, image.callback)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    rospy.Service('vision/path',
                  VisionSrvPath(), mission_callback)
    output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
