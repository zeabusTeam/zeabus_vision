#!/usr/bin/python2.7
"""
    File name: auto_exposure_bottom.py
    Author: skconan
    Maintainer: AyumiizZ
    Date created: 2018/11/04
    Python Version: 2.7
"""


import rospy
from auto_exposure import AutoExposure


def auto_exposure_bottom():
    rospy.init_node('AutoExposureBottom')
    AE = AutoExposure("/auto_exposure_bottom", debug=False)
    AE.run()


if __name__ == '__main__':
    auto_exposure_bottom()
