#!/usr/bin/python2.7
"""
    File name: auto_exposure_front.py
    Author: skconan
    Maintainer: AyumiizZ
    Date created: 2018/11/03
    Python Version: 2.7
"""


import rospy
from auto_exposure import AutoExposure
from zeabus_utility.srv import VisionSrvAE

AE = AutoExposure("/auto_exposure_front", debug=False)

def callback(msg):
    if str(msg.request.data) == 'front_ae':
        AE.change_and_wait()
        return True

def auto_exposure_front():
    rospy.init_node('AutoExposureFront')
    rospy.Service('vision/ae/front', VisionSrvAE(), callback)
    AE.run()


if __name__ == '__main__':
    auto_exposure_front()
    rospy.spin()
