#! /usr/bin/python2

from __future__ import division
from __future__ import print_function
import cv2


class GateCheck:

    def predict(self, ct, img=None):
        x, y, w, h = cv2.boundingRect(ct)
        (cx, cy), (w, h), angle = cv2.minAreaRect(ct)

        ct_area = cv2.contourArea(ct)
        objAreaRatio = ct_area/float(w*h)
        # ratioCond1 = False or ((h*2 > 0.75*w) and (h < 1.25*w))
        # ratioCond2 = ((h*2 > 0.75*w) and (h*2 < 1.25*w))
        ratioCond1 = False and ((h*2 > 0.75*w) and (h < 1.25*w))
        # ratioCond2 = ((w/h > 4.5) and (w/h < 11.5))
        ratioCond2 = ((w/h > 4.5) and (w/h < 15)) and abs(angle) < 45
        heightCond = True
        sizeCond = True
        if img is not None:
            # heightCond = y/img.shape[0] < 0.7
            sizeCond = (float(w*h)/img.shape[0]/img.shape[1]) > 0.008 and (
                float(w*h)/img.shape[0]/img.shape[1]) < 0.50
        condLeg = True
        # cropped = self.cropFour(img, ct)
        # condLeg = self.condLeg(cropped)
        sumCond = objAreaRatio > 0.1 and objAreaRatio < 0.8 and (
            ratioCond1 or ratioCond2) and condLeg and sizeCond and heightCond
        debug = {
            'objAreaRatio': objAreaRatio,
            'ratioCond1': ratioCond1,
            'ratioCond2': ratioCond2,
            'heightCond': heightCond,
            'sizeCond': sizeCond,
            'condLeg': condLeg,
            'angle': angle,
            'w,h': (w, h)
        }
        # print(debug)
        if sumCond:
            return 1
        return 0

    def cropFour(self, img, ct):
        cropped = []
        x, y, w, h = cv2.boundingRect(ct)
        y = y+h-w/2
        h = w/2
        y = int(y+h/4)
        for i in range(4):
            cropped.append(img[int(y+h/4):int(y+h*3/4),
                               int(x+i*w/4):int(x+(i+1)*w/4)])
        return cropped

    def condLeg(self, cropped):
        for c in cropped:
            if c.size < 100:
                return False
        cond = True
        first = cropped[0].copy()
        first = first/255
        cond = cond and (first.sum()/first.size > 0)
        # cv2.imshow('first', cropped[0])
        # print(first.sum()/first.size, end=' ')
        fourth = cropped[3].copy()
        fourth = fourth/255
        cond = cond and (fourth.sum()/fourth.size > 0)
        # cv2.imshow('fourth', cropped[3])
        # print(fourth.sum()/fourth.size, end='   ')
        second = cropped[1].copy()
        second = second/255
        secondCond = second.sum()/second.size > 0
        # cv2.imshow('second', cropped[1])
        # print(second.sum()/second.size)
        third = cropped[2].copy()
        third = third/255
        thirdCond = third.sum()/second.size > 0
        # cv2.imshow('third', cropped[1])
        # print(third.sum()/third.size)
        cond = cond and ((not thirdCond and secondCond)
                         or (not secondCond and thirdCond))
        cv2.waitKey(0)
        return cond
