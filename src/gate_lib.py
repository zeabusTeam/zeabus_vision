from __future__ import division
from __future__ import print_function
import cv2
import numpy as np


class Gate:
    ''' GATE Processing class.
    You can use this class to process ROBOSUB gate mission.
    Parameters:
        fileOrDevice (str,int): you can send this to OpenCV to open
    '''

    def __init__(self, fileToOpen=None):
        if fileToOpen is not None:
            self.device = cv2.VideoCapture(fileToOpen)
        else:
            self.device = None
        self.filename = fileToOpen
        self.clahe = cv2.createCLAHE(clipLimit=256, tileGridSize=(13, 13))
        self.okrange = 20
        self.okval = 256

    def adjustVal(self, th):
        self.okval = th

    def adjustRange(self, th):
        self.okrange = th

    def read(self):
        '''Read opedgesenned file and openImg Window
        '''
        cv2.namedWindow(str(self.filename)+' ct')
        cv2.createTrackbar('val', str(self.filename)+' ct',
                           self.okval, 256, self.adjustVal)
        cv2.createTrackbar('TileGridSize', str(self.filename)+' ct',
                           self.okrange, 256, self.adjustRange)
        read = True
        while self.device.isOpened():
            if read:
                retval, image = self.device.read()
            if not retval:
                break
            self.doProcess(image, True)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            if key == ord('s'):
                read = not read

    def doProcess(self, img, showImg=False):
        """Put image then get outputs

        Arguments:
            img {OpenCV Image} -- Input image

        Keyword Arguments:
            showImg {bool} -- Wanna Show Img for debugging ? (default: {False})

        Returns:
            list -- Found data. None or list of cx1,cy1,cx2,cy2,area
        """
        img = cv2.resize(img, None, fx=0.50, fy=0.50)
        processed = self._process(img)
        if showImg:
            cv2.imshow(str(self.filename)+' raw', img)
            cv2.imshow(str(self.filename)+' ct', processed[1])
            cv2.imshow(str(self.filename)+' 2', processed[2])
        # if processed[6] is not None:
        #     diff = self.calcDiffPercent(processed[6], self.last_detect)
        #     cond = self.last_detect is None or diff[0] < 0.2
        #     if cond:
        #         self.last_detect = processed[6]
        return (processed[3], img, processed[2])

    def _process(self, img):
        # img = cv2.resize(img, None, fx=0.5,fy=0.5)
        # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        interest0 = cv2.extractChannel(img, 0)*0.05
        interest1 = cv2.extractChannel(img, 1)*0.6
        interest2 = cv2.extractChannel(img, 2)*0.35
        interest = np.array(interest0+interest1+interest2, np.uint8)
        interest = self.clahe.apply(interest)
        # cv2.imshow('interest', interest)
        mask = cv2.inRange(interest, self.okval-self.okrange,
                           self.okval+self.okrange)
        kernel = np.ones((7, 7), np.uint8)
        noise_removed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        noise_removed = cv2.medianBlur(noise_removed, 9)
        _, cts, hi = cv2.findContours(
            noise_removed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        def getHigh(ct):
            x, y, w, h = cv2.boundingRect(ct)
            return h

        def getPos(rect):
            return rect[0]

        cts = sorted(cts, key=getHigh, reverse=True)

        three_legs = []

        for ct in cts:
            x, y, w, h = cv2.boundingRect(ct)
            if len(three_legs) >= 3:
                break
            if w < h/5 and w > h/10:
                cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 0), 3)
                three_legs.append((x, y, w, h))

        three_legs = sorted(three_legs, key=getPos)

        if len(three_legs) == 2:
            three_legs.append(None)
            if three_legs[0][3]/three_legs[1][3] >= 0.3 and three_legs[0][3]/three_legs[1][3] <= 0.7:
                three_legs[2] = three_legs[1]
                three_legs[1] = three_legs[0]
                three_legs[0] = None
            elif three_legs[0][3]/three_legs[1][3] >= 0.75 and three_legs[0][3]/three_legs[1][3] <= 1.25:
                three_legs[2] = three_legs[1]
                three_legs[1] = None

        cx = 0
        cy = 0
        x_left = -1
        x_right = -1
        _found = None
        if len(three_legs) > 1:
            if len(three_legs) == 1:
                cx = three_legs[0][0]+three_legs[0][2]/2
                cy = three_legs[0][1]+three_legs[0][3]/2
            if len(three_legs) == 2:
                cx = three_legs[0][0]+three_legs[0][2]/2+three_legs[0][3]
                cy = three_legs[0][1]+three_legs[0][3]/2
            if len(three_legs) == 3:
                if three_legs[0] is not None and three_legs[2] is not None:
                    cx += three_legs[0][0]+three_legs[0][2]/2
                    cy += three_legs[0][1]+three_legs[0][3]/2
                    cx += three_legs[2][0]+three_legs[2][2]/2
                    cy += three_legs[2][1]+three_legs[2][3]/2
                    cx /= 2
                    cy /= 2
                else:
                    if three_legs[0] is None:
                        cx = three_legs[2][0] + \
                            three_legs[2][2]/2-three_legs[2][3]
                        cy = three_legs[2][1]+three_legs[2][3]/2
                    if three_legs[2] is None:
                        cx = three_legs[0][0] + \
                            three_legs[0][2]/2+three_legs[0][3]
                        cy = three_legs[0][1]+three_legs[0][3]/2
            cv2.circle(img, (int(cx), int(cy)), 10, (255, 255, 0), 3)
            if three_legs[0] is not None:
                x_left = three_legs[0][0]
            if three_legs[2] is not None:
                x_right = three_legs[2][0]+three_legs[2][2]
            if x_left == -1:
                three_legs[2][0]+three_legs[2][2]/2-three_legs[2][3]
            if x_right == -1:
                three_legs[0][0]+three_legs[0][2]/2+three_legs[0][3]

            if len(three_legs) == 3 and None not in three_legs:
                x, y, w, h = three_legs[1]
                fourty_range = h*2.5
                min_left = three_legs[0][0]+three_legs[0][2]
                min_right = three_legs[2][0]
                if x - fourty_range < min_left:  # fourty is left
                    if x + w + fourty_range < min_right:
                        return (img, mask, noise_removed, None)
                elif x + w + fourty_range > min_right:  # fourty is right
                    if x - fourty_range > min_left:
                        return (img, mask, noise_removed, None)

            if len(three_legs) == 3 and three_legs[1] is None:
                x1, y1, w1, h1 = three_legs[0]
                x2, y2, w2, h2 = three_legs[2]
                if x2-x1 > (h1+h2)*1.1 or x2-x1 < (h1+h2)*0.9:
                    return (img, mask, noise_removed, None)
            _found = (cx/img.shape[1]*2-1, cy/img.shape[0]*2-1,
                      x_left/img.shape[1]*2-1, x_right/img.shape[1]*2-1,
                      1, (x_right-x_left)/img.shape[1]*2-1)
        return (img, mask, noise_removed, _found)

    def calcDiffPercent(self, first, second):
        if first is None or second is None or len(first) < len(second):
            return [0]
        res = []
        for key, val in enumerate(first):
            res.append(abs(val-second[key])/2)
        return tuple(res)
