#!/usr/bin/python2.7
"""
    File name: gen-data-vision.py
    Author: AyumiizZ
    Date created: 2019/9/16
    Python Version: 2.7
"""
import rospy
from random import randint, randrange
from zeabus_vision.msg import TestVisionData, TestMpData

pub = rospy.Publisher('/test/mp/data', TestMpData, queue_size=3)
Min = 0.2
Max = 2.5


def message(x=0, y=0, z=0):
    msg = TestMpData()
    msg.header.stamp = rospy.Time.now()
    msg.target = [x,y,z] + [0]*3
    print('----Pub----')
    print(msg)
    return msg

def trimmed(data):
    raw_data = float(data)
    data = abs(raw_data)
    sign = 1 if raw_data >= 0 else -1
    if data > 0:
        if data > Max:
            return sign*Max
        elif data < Min:
            return sign*Min
    return data


def mission_callback(msg):
    x = trimmed(msg.point1.x) if abs(msg.point1.x) > 7 else 0
    y = trimmed(msg.point1.y) if abs(msg.point1.y) > 7 else 0
    z = trimmed(msg.point1.z) if abs(msg.point1.z) > 7 else 0
    pub.publish(message(x,y,z))


if __name__ == "__main__":
    rospy.init_node('gen_data_mp')
    rospy.Subscriber("/test/vision/data", TestVisionData,
                     callback=mission_callback, queue_size=3)
    rospy.spin()
