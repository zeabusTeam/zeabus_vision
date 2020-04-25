#!/usr/bin/python2.7
"""
    File name: example-publisher.py
    Author: AyumiizZ
    Date created: 2020/04/25
    Python Version: 2.7
"""
import rospy
import zbuspy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage
from vision_lib import ImageTools

image = ImageTools(sub_sampling=0.3)
publisher = zbuspy.Publisher(port=6969, datatype=zbuspy.msgs.OPENCVIMAGE)

if __name__ == "__main__":
    rospy.init_node('example_publisher')
    rospy.Subscriber(image.topic.FRONT, CompressedImage,
                     callback=image.callback, queue_size=3)
    img_is_none = True
    while not rospy.is_shutdown():
        if image.bgr is None:
            print('image is none')
            rospy.sleep(0.05)
            continue
        if img_is_none == True:
            print('have image')
        img_is_none = False
        publisher.publish(image.bgr)
        rospy.sleep(0.05)
    rospy.spin()
