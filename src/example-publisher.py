#!/usr/bin/python2.7
"""
    File name: example-publisher.py
    Author: AyumiizZ
    Date created: 2020/04/25
    Python Version: 2.7
"""
import rospy
import time
import zbuspy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage
from vision_lib import ImageTools

image = ImageTools(sub_sampling=0.3)
publisher = zbuspy.Publisher(port=6969, datatype=zbuspy.msgs.OPENCVIMAGE)
image_count = 0
if __name__ == "__main__":
    rospy.init_node('example_publisher')
    rospy.Subscriber(image.topic.FRONT, CompressedImage,
                     callback=image.callback, queue_size=3)
    img_is_none = True
    service_time = []
    while not rospy.is_shutdown():
        if image.bgr is None:
            print('image is none')
            rospy.sleep(0.05)
            continue
        if img_is_none == True:
            print('have image')
        start = time.time()
        img_is_none = False
        image_count += 1
        # resized = cv.resize(image.bgr, (416,416), interpolation = cv.INTER_AREA)
        resized = cv.resize(image.bgr, (600,600), interpolation = cv.INTER_AREA)
        publisher.publish(resized)
        # publisher.publish(image.bgr)
        service_time.append(1.0/(time.time()-start))
        # rospy.sleep(0.0001) 
        # print(image_count)
        if image_count % 250 == 0:
            print('image_count: {}'.format(image_count))
        if image_count >= 5e3:
            service_time = sorted(service_time)
            Q1 = service_time[(len(service_time)+1)/4]
            Q3 = service_time[3*(len(service_time)+1)/4]
            iqr = Q3-Q1
            lower_outlier = Q1 - 1.5*iqr
            upper_outlier = Q3 + 1.5*iqr
            print(Q1, Q3, lower_outlier, upper_outlier)
            service_time = [i for i in service_time if i <= upper_outlier and i >= lower_outlier]
            break
    
    print(len(service_time))
    print('Max: {}\nMin: {}\nAvg: {}'.format(max(service_time), min(service_time), sum(service_time)/len(service_time)))
    rospy.spin()