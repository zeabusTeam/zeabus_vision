#!/usr/bin/python2.7
"""
    File name: gen-graph.py
    Author: AyumiizZ
    Date created: 2019/9/16
    Python Version: 2.7
"""

import os
import rospy
import cv2 as cv
import numpy as np
from time import sleep
from zeabus_vision.msg import TestMpData, TestVisionData
import matplotlib.pyplot as plt

class Data:
    def __init__(self):
        self.time = []
        self.x = []
        self.y = []
        self.z = []

    def append(self,x=0,y=0,z=0,time=0):
        self.time.append(time.secs%10000 + 10e-10*time.nsecs)  
        # print(10e-9*time.nsecs)  
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)

        

mp_data = Data()
v_data = Data()

def mp_callback(msg):
    x,y,z = list(msg.target)[:3]
    x*=40
    y*=40
    z*=40
    mp_data.append(x,y,z,time=msg.header.stamp)

def vision_callback(msg):
    x = msg.point1.x
    y = msg.point1.y
    z = msg.point1.z
    v_data.append(x,y,z,time=msg.header.stamp)

if __name__ == "__main__":
    rospy.init_node('gen_graph')
    rospy.Subscriber("/test/vision/data", TestVisionData,
                     callback=vision_callback, queue_size=3)
    rospy.Subscriber("/test/mp/data", TestMpData,
                     callback=mp_callback, queue_size=3)
    while not rospy.is_shutdown():
        # print(mp_data.time, v_data.time)
        if (mp_data.time == [] or v_data.time == []):
            continue
        max_time = max(mp_data.time[-1],v_data.time[-1])
        print(mp_data.time)
        print(max_time)
        plt.subplot(311)
        plt.xlim(max_time-10,max_time)
        plt.ylim(-105,105)
        plt.plot(mp_data.time,mp_data.x,color='red')
        plt.plot(v_data.time,v_data.x,color='blue')
        plt.subplot(312)
        plt.xlim(max_time-10,max_time)
        plt.ylim(-105,105)
        plt.plot(mp_data.time,mp_data.y,color='red')
        plt.plot(v_data.time,v_data.y,color='blue')
        plt.subplot(313)
        plt.xlim(max_time-10,max_time)
        plt.ylim(-105,105)
        plt.plot(mp_data.time,mp_data.z,color='red')
        plt.plot(v_data.time,v_data.y,color='blue')
        # histr = cv.calcHist([gray], [0], None, [256], [0, 256])
        # plt.plot(histr, color='blue')

        plt.pause(0.01)
        plt.subplot(311)
        plt.clf()
        plt.subplot(312)
        plt.clf()
        plt.subplot(313)
        plt.clf()
        
    rospy.spin()