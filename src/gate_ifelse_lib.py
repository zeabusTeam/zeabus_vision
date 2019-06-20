#! /usr/bin/python2

import cv2


class GateCheck:

    def predict(self, ct, img_size=None):
        x, y, w, h = cv2.boundingRect(ct)
        ct_area = cv2.contourArea(ct)
        objAreaRatio = ct_area/float(w*h)
        ratioCond = ((h*2 > 0.9*w) and (h*2 < 1.1*w)
                     ) or ((h > 0.85*w) and (h < 1.15*w))
        if img_size is None:
            sizeCond = True
        else:
            sizeCond = (float(w*h)/img_size) > 0.025
        if objAreaRatio < 0.3 and ratioCond and sizeCond:
            return 1
        return 0
