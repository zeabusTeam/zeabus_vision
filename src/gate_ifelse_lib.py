#! /usr/bin/python2

from __future__ import division
import cv2


class GateCheck:

    def predict(self, ct, img_shape=None):
        x, y, w, h = cv2.boundingRect(ct)
        ct_area = cv2.contourArea(ct)
        objAreaRatio = ct_area/float(w*h)
        ratioCond = ((h*2 > 0.9*w) and (h*1 < 1.15*w))
        heightCond = True
        sizeCond = True
        if img_shape is not None:
            heightCond = y/img_shape[0] < 0.5
            sizeCond = (float(w*h)/img_shape[0]/img_shape[1]) > 0.025 and (
                float(w*h)/img_shape[0]/img_shape[1]) < 0.80
        if objAreaRatio < 0.3 and ratioCond and sizeCond and heightCond:
            return 1
        return 0
