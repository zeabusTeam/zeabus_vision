#!/usr/bin/python2.7
"""
    File name: task_process.py
    Author: AyumiizZ
    Date created: 2019/10/22
    Python Version: 2.7
"""
import rospy
from random import randint, randrange
import numpy as np
import cv2 as cv


class Task:
    def __init__(self, task):
        self.task = task
        self.bound = self.get_bound()

    def get_bound(self):
        color = {
            'red': [0, 0, 255],
            'green': [0, 255, 0],
            'blue': [255, 0, 0]
        }
        if self.task in color.keys():
            upper = lower = np.array(color[self.task], dtype=np.uint8)
        else:
            upper = np.array([0, 0, 0], dtype=np.uint8)
            lower = np.array([255, 255, 255], dtype=np.uint8)

    def process(self, img, debug=False):
        mask = cv.inRange(img, self.bound, self.bound)

        kernel = np.ones((5, 5), np.uint8)
        dilated = cv.dilate(mask, kernel, iterations=2)
        _, dilated = cv.threshold(dilated, 127, 255, type=cv.THRESH_BINARY)
        _, contours, hierarchy = cv.findContours(
            dilated, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        result_image = np.zeros(img.shape)
        result_data = []
        for cnt, hrc in zip(contours, hierarchy[0]):
            if debug:
                x, y, w, h = cv.boundingRect(cnt)
                cv.rectangle(result_image, (x, y), (x+w, y+h), (0, 0, 255), 1)
            if(hrc[3] == -1):  # if it is parent
                x, y, w, h = cv.boundingRect(cnt)
                cv.rectangle(result_image, (x, y), (x+w, y+h), (0, 255, 0), 1)
                '''
                result_data point
                1                 2 
                  +-+-+-+-+-+-+-+
                  |   result    |
                  +-+-+-+-+-+-+-+
                4                 3
                '''
                pt1, pt2, pt3, pt4 = (x, y), (x+w, y), (x+w, y+h), (x, y+h)
                result_data.append((pt1, pt2, pt3, pt4))
        return result_data
