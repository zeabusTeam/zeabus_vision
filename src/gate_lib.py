from __future__ import division
import time
import cv2
import numpy as np
from gate_ml_lib import GateML
from gate_ifelse_lib import GateCheck


class Gate:
    ''' GATE Processing class.
    You can use this class to process ROBOSUB gate mission.
    Parameters:
        fileOrDevice (str,int): you can send this to OpenCV to open
    '''

    useml = False

    def __init__(self, fileToOpen=None):
        self.GateML = GateML()
        self.GateCond = GateCheck()
        if fileToOpen is not None:
            self.device = cv2.VideoCapture(fileToOpen)
        else:
            self.device = None
        self.filename = fileToOpen
        self.last_detect = None
        self.widthStack = []
        self.lastTime = None

    def read(self):
        '''Read opedgesenned file and openImg Window
        '''

        while self.device.isOpened():
            retval, image = self.device.read()
            if not retval:
                break
            self.doProcess(image, True)
            key = cv2.waitKey(50)
            if key == ord('q'):
                break

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
            # cv2.imshow(str(self.filename)+' ct', processed[1])
            # cv2.imshow(str(self.filename)+' 2', processed[2])
            # cv2.imshow(str(self.filename)+' 3', processed[3])
            # cv2.imshow(str(self.filename)+' 4', processed[4])
            cv2.imshow(str(self.filename)+' 5', processed[5])
        if processed[6] is not None:
            diff = self.calcDiffPercent(processed[6], self.last_detect)
            cond = self.last_detect is None or diff[0] < 0.2
            if cond:
                self.last_detect = processed[6]
        return (processed[6], processed[5], processed[4])

    def _process(self, img):
        def my_area(ct):
            x, y, w, h = cv2.boundingRect(ct)
            return w*h
        self.img_size = img.shape[0:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        blur_k = int(self.img_size[0]/150)
        blur_k += (blur_k+1) % 2
        noise_removed = cv2.medianBlur(gray, blur_k)
        ret, th1 = cv2.threshold(
            noise_removed, 20, 255, cv2.THRESH_BINARY_INV)
        th3 = cv2.adaptiveThreshold(noise_removed,
                                    255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY_INV, blur_k*4+1, 2)
        bw_th3 = cv2.bitwise_and(th1, th3)
        kernel = np.ones((blur_k*2, blur_k*2), np.uint8)
        closing = cv2.morphologyEx(bw_th3, cv2.MORPH_CLOSE, kernel)
        _, cts, hi = cv2.findContours(
            closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cts = sorted(cts, key=my_area, reverse=True)
        self.temp_img = img
        filtered = self.FindGateFromGates(cts, gray)
        found = None
        withct = img.copy()
        for c in filtered:
            x, y, w, h = cv2.boundingRect(c)
            c_area = cv2.contourArea(c)
            _found = ((2*x+w)/img.shape[1]-1, (2*y+h)/img.shape[0] - 1,
                      2*x/img.shape[1]-1, 2*(x+w)/img.shape[1]-1,
                      c_area/w/h)
            if self.lastTime is None or time.time()-self.lastTime < 10:
                diff = self.calcDiffPercent(_found, self.last_detect)
                cond = self.last_detect is None or (diff[0] < 0.2)
                if len(self.widthStack) > 0:
                    avgWidth = sum(self.widthStack)/len(self.widthStack)
                    acceptable = 0.4
                    widthCond = (_found[3]-_found[2]) < avgWidth + \
                        acceptable and (_found[3]-_found[2]) > avgWidth-acceptable
                    cond = cond and acceptable
            else:
                cond = True
            if cond:
                found = _found
                self.widthStack.append(found[3]-found[2])
                cv2.rectangle(withct, (x, y), (x+w, y+h), (255, 255, 0), 3)
                cv2.circle(withct, (int(x+w/2), int(y+h/2)),
                           4, (0, 255, 255), 4)
                break
        return (img, noise_removed, th1, bw_th3, closing, withct, found)

    def calcDiffPercent(self, first, second):
        if first is None or second is None or len(first) < len(second):
            return [0]
        res = []
        for key, val in enumerate(first):
            res.append(abs(val-second[key])/2)
        return tuple(res)

    def prepareData(self, gray):
        data_t = cv2.resize(gray, (40, 20))
        return data_t

    def FindGateFromGates(self, cts, gray):
        # eqlst = cv2.equalizeHist(gray)
        # opt = self.temp_img.copy()
        if len(cts) > 10:
            cts = cts[:10]
        outputs = []
        for i, ct in enumerate(cts):
            x, y, w, h = cv2.boundingRect(ct)
            if self.useml:
                only_ct = np.zeros_like(gray, 'uint8')
                cv2.fillPoly(only_ct, pts=[ct], color=255)
                mini = only_ct[y:y+h, x:x+w]
                prepared = self.prepareData(mini)
                if self.GateML.predict(prepared) == 1:
                    # color = (0, 255, 0)
                    outputs.append(ct)
            else:
                if self.GateCond.predict(ct, gray.shape) == 1:
                    # color = (0, 255, 0)
                    outputs.append(ct)
            # else:
            #     color = (255, 0, 255)
            # cv2.rectangle(opt, (x, y), (x+w, y+h), color, 2)
            # cv2.putText(opt, str(i), (int(x+w/2), int(y+h/2)),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
            # cv2.imshow(str(i)+' mini', prepared)
        # cv2.imshow('ML', opt)
        # key = cv2.waitKey(0)
        # for i, ct in enumerate(cts):
        #     x, y, w, h = cv2.boundingRect(ct)
        #     mini = gray[y:y+h, x:x+w]
        #     prepared = self.prepareData(mini)
        #     if chr(key) == str(i):
        #         self.thisIsGate(prepared)
        #     else:
        #         self.thisIsNotGate(prepared)
        return outputs
