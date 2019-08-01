#!/usr/bin/python2.7
"""
    File name: auto_exposure_hist.py
    Author: skconan
    Maintainer: AyumiizZ
    Date created: 2018/10/16
    Python Version: 2.7
"""

import os
import rospy
import cv2 as cv
import numpy as np
from time import sleep, time
import matplotlib.pyplot as plt
from vision_lib import Statistics
from sensor_msgs.msg import CompressedImage
from dynamic_reconfigure.client import Client as Client
from zeabus_utility.srv import VisionSrvAE


class Log:
    def __init__(self):
        self.history = []

    def append(self, bias):
        if bias > 0:
            pos = 'up'
        elif bias == 0:
            pos = '-'
        elif bias < 0:
            pos = 'down'
        self.history.append(pos)
        print(self.history)
        if len(self.history) > 12:
            self.history = self.history[-12:]

    def sleep(self, time):
        if self.history == ['down'] * 10 + ['up'] * 2:
            print('sleep')
            sleep(time)


class AutoExposure:
    def __init__(self, namespace, debug=False):
        print(namespace)
        self.log = Log()
        image_topic = rospy.get_param(namespace + "/topic")
        self.client_name = rospy.get_param(namespace + "/client")
        self.p_lower_nth_default = rospy.get_param(
            namespace + '/percentile_nth_min')
        self.p_upper_nth_default = rospy.get_param(
            namespace + '/percentile_nth_max')

        self.p_lower_default = rospy.get_param(namespace + '/percentile_lower')
        self.p_upper_default = rospy.get_param(namespace + '/percentile_upper')

        self.exposure_default = rospy.get_param(
            namespace + '/exposure_default')

        self.p_lower_nth_default = int(self.p_lower_nth_default)
        self.p_upper_nth_default = int(self.p_upper_nth_default)
        self.p_upper_default = int(self.p_upper_default)
        self.p_lower_default = int(self.p_lower_default)
        self.debug = debug

        print("Topic: " + str(image_topic))
        print("Client: " + str(self.client_name))
        print("Percentile nth min: " + str(self.p_lower_nth_default))
        print("Percentile nth max: " + str(self.p_upper_nth_default))
        print("Percentile lower: " + str(self.p_lower_default))
        print("Percentile upper: " + str(self.p_upper_default))
        print("Exposure_default: " + str(self.exposure_default))

        rospy.Subscriber(image_topic, CompressedImage, self.image_callback)
        self.PATH_PKG = os.path.dirname(os.path.abspath(__file__))
        self.image = None
        self.client = Client(self.client_name)
        self.sub_sampling = 0.25
        self.stat = Statistics()

        self.wait_time = time()
        self.ros_auto = False

    def change_and_wait(self, wait_time=5):
        params = {"auto_exposure": bool(True)}
        self.client.update_configuration(params)
        self.wait_time = time() + wait_time
        self.ros_auto = True
        rospy.sleep(0.2)

    def get_change_back(self):
        params = {"auto_exposure": bool(False)}
        self.client.update_configuration(params)
        self.ros_auto = False

    def image_callback(self, msg):
        arr = np.fromstring(msg.data, np.uint8)
        self.image = cv.resize(cv.imdecode(arr, 1), (0, 0),
                               fx=self.sub_sampling, fy=self.sub_sampling)

    def nothing(self, x):
        pass

    def set_param(self, value=0.1):
        params = {"exposure": float(value)}
        self.client.update_configuration(params)

    def get_exposure(self):
        value = rospy.get_param("/" + self.client_name + "/exposure", None)
        return value

    def pre_processing(self, image):
        blur = cv.blur(image.copy(), (11, 11))
        return blur

    def run(self):
        if self.debug:
            cv.namedWindow('image')
            cv.moveWindow('image', 100, 100)

            cv.createTrackbar('p_lower', 'image', 0, 50, self.nothing)
            cv.createTrackbar('p_upper', 'image', 50, 100, self.nothing)

            cv.createTrackbar('th_p_lower', 'image', 0, 255, self.nothing)
            cv.createTrackbar('th_p_upper', 'image', 0, 255, self.nothing)

            cv.setTrackbarPos('p_lower', 'image', self.p_lower_nth_default)
            cv.setTrackbarPos('p_upper', 'image', self.p_upper_nth_default)
            cv.setTrackbarPos('th_p_lower', 'image', self.p_lower_default)
            cv.setTrackbarPos('th_p_upper', 'image', self.p_upper_default)

        p_upper = self.p_upper_nth_default
        p_lower = self.p_lower_nth_default

        th_p_lower = self.p_lower_default
        th_p_upper = self.p_upper_default
        previous_ev = self.exposure_default

        while not rospy.is_shutdown():
            if self.image is None:
                continue

            if self.wait_time > time():
                rospy.sleep(0.1)
                continue

            if self.ros_auto:
                self.get_change_back()

            try:
                pre_process_bgr = self.pre_processing(self.image)
                gray = cv.cvtColor(pre_process_bgr, cv.COLOR_BGR2GRAY)

                current_ev = self.get_exposure()

                if current_ev is None:
                    ev = previous_ev
                else:
                    ev = current_ev

                if self.debug:
                    p_lower = cv.getTrackbarPos('p_lower', 'image')
                    p_upper = cv.getTrackbarPos('p_upper', 'image')

                    th_p_lower = cv.getTrackbarPos('th_p_lower', 'image')
                    th_p_upper = cv.getTrackbarPos('th_p_upper', 'image')

                    cv.imshow('gray', gray)
                    cv.imshow('image', self.image)
                    cv.imshow('pre_process_bgr', pre_process_bgr)

                    k = cv.waitKey(1) & 0xff
                    if k == ord('q'):
                        break
                    histr = cv.calcHist([gray], [0], None, [256], [0, 256])
                    plt.plot(histr, color='blue')

                    plt.pause(0.00001)
                    plt.clf()

                current_p_lower = self.stat.get_percentile(gray, p_lower)
                current_p_upper = self.stat.get_percentile(gray, p_upper)

                print('==' * 20)
                print(th_p_lower, th_p_upper, ev)
                print(current_p_lower, current_p_upper, ev)

                try:
                    if not self.ros_auto:
                        if current_p_lower < th_p_lower:
                            self.set_param(ev + 0.04)
                        if current_p_upper > th_p_upper:
                            self.set_param(ev - 0.04)
                except:
                    print('cannot set param')

                self.log.append(current_ev-previous_ev)

                previous_ev = current_ev
                self.log.sleep(10)
            except:
                print('what that bug')
        if self.debug:
            plt.close()
            cv.destroyAllWindows()
