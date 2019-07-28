from __future__ import division
import cv2
import os
import numpy as np
import time
# from buoy_predict import BuoyML


class VisionException(Exception):
    pass


class SourceIsNotOpened(VisionException):
    pass


class BuoyReturn:
    def __init__(self):
        self.time = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.score = 0
        self.area = -1
        self.result_img = None
        self.cropped_img = None

    def __str__(self):
        this = self.__dict__.copy()
        # print(this.keys())
        del this['result_img']
        del this['cropped_img']
        return str(this)


class Buoy:

    lockX = 800
    MIN_POINTS = 5

    SOURCE_TYPE = {
        'FILE': 0,
        'SINGLE_IMG': 1,
        'WEBCAM': 2,
        'SEG_IMG': 3,
    }

    LASTFOUND = None

    OPENED = False
    OPENED_TYPE = None

    def __init__(self, rospy):

        self.rospy = rospy

        # Load ML Lib
        # self.ML = BuoyML()

        # Load Ref img
        filedir = os.path.dirname(os.path.abspath(__file__))
        self.jiangshi = cv2.imread(os.path.join(
            filedir, 'pictures', 'jiangshi.png'), 0)
        # self.jiangshi = cv2.resize(self.jiangshi, None, fx=0.1, fy=0.1)
        # self.jiangshi = cv2.medianBlur(self.jiangshi, 7)
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.ori_kp, self.ori_des = self.sift.detectAndCompute(
            self.jiangshi, None)

        # Init FLANN
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=8)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        # self.flann = cv2.BFMatcher()

    def openSource(self, sourceType, source=0):
        if sourceType == self.SOURCE_TYPE['FILE']:
            self.cv_dev = cv2.VideoCapture(source)
        if sourceType == self.SOURCE_TYPE['SINGLE_IMG']:
            self.img = source.copy()
        if sourceType == self.SOURCE_TYPE['SEG_IMG']:
            self.img = source[0].copy()
            self.img_seg = source[1].copy()
        if sourceType == self.SOURCE_TYPE['WEBCAM']:
            self.cv_dev = cv2.VideoCapture(source)
        if sourceType in self.SOURCE_TYPE.values():
            self.OPENED = True
            self.OPENED_TYPE = sourceType

    def read(self):
        if not self.OPENED:
            raise SourceIsNotOpened
        if self.OPENED_TYPE == self.SOURCE_TYPE['FILE']:
            _, self.img = self.cv_dev.read()
        if self.OPENED_TYPE == self.SOURCE_TYPE['SINGLE_IMG']:
            return True
        if self.OPENED_TYPE == self.SOURCE_TYPE['SEG_IMG']:
            return True
        if self.OPENED_TYPE == self.SOURCE_TYPE['WEBCAM']:
            _, self.img = self.cv_dev.read()
        return _

    def preprocess(self):
        img = self.img.copy()
        zoom_rate = self.lockX/img.shape[1]
        self.img_sm = cv2.resize(img, None, fx=zoom_rate, fy=zoom_rate)
        if self.OPENED_TYPE == self.SOURCE_TYPE['SEG_IMG']:
            img_seg = self.img_seg.copy()
            zoom_rate = self.lockX/img_seg.shape[1]
            self.img_seg_sm = cv2.resize(
                img_seg, None, fx=zoom_rate, fy=zoom_rate)
        # img_lab = cv2.cvtColor(self.img_sm, cv2.COLOR_BGR2LAB)
        # img_lab[:, :, 0] = cv2.equalizeHist(img_lab[:, :, 0])
        # self.img_sm = cv2.cvtColor(img_lab, cv2.COLOR_LAB2BGR)
        self.img_sm = cv2.GaussianBlur(self.img_sm, (3, 3), 256)
        self.img_gray = cv2.cvtColor(self.img_sm, cv2.COLOR_BGR2GRAY)

    def process_seg(self):
        result = BuoyReturn()
        # lower = self.rospy.get_param(
        #     "/object_detection/object_color_range/buoy/lower")
        # upper = self.rospy.get_param(
        #     "/object_detection/object_color_range/buoy/upper")
        # lower = [int(lp) for lp in lower.split(',')]
        # upper = [int(lp) for lp in upper.split(',')]
        # lower = np.array(lower, np.uint8)
        # upper = np.array(upper, np.uint8)
        hsv = cv2.cvtColor(self.img_seg_sm, cv2.COLOR_BGR2HSV)
        s = cv2.extractChannel(hsv, 1)
        ret, mask = cv2.threshold(
            s, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        # mask = cv2.inRange(hsv, (0, 0, 0), (10, 30, 150))
        kernel = np.ones((10, 20), np.uint8)
        thismask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # thismask = 255 - thismask
        _, cts, hi = cv2.findContours(
            mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cts = sorted(cts, key=cv2.contourArea, reverse=True)
        for ct in cts:
            thismask = np.zeros_like(self.img_sm)
            cv2.drawContours(thismask, [ct], -1, 255, -1)
            out = np.zeros_like(self.img_sm)
            out[thismask == 255] = self.img_sm[thismask == 255]
            # cv2.imshow('out', out)
            # cv2.waitKey(0)
            x, y, w, h = cv2.boundingRect(ct)
            obj = out[y:y+h, x:x+w]
            gray_crop = cv2.cvtColor(obj, cv2.COLOR_BGR2GRAY)
            _, th = cv2.threshold(
                gray_crop, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            circles = cv2.HoughCircles(
                th, cv2.HOUGH_GRADIENT, 1, 20,
                param1=50, param2=10, maxRadius=int(obj.shape[0]/10))
            if circles is not None and cv2.contourArea(ct)/self.img_sm.shape[0]/self.img_sm.shape[1] < 0.6:
                cv2.rectangle(self.img_sm, (x, y),
                              (x+w, y+h), (0, 255, 255), 3)
                cv2.circle(self.img_sm, (int(x+w/2), int(y+h/2)),
                           3, (0, 255, 255), 3)
                result.result_img = self.img_sm
                result.score = 1
                result.time = time.time()
                result.area = cv2.contourArea(
                    ct)/self.img_sm.shape[0]/self.img_sm.shape[1]
                result.cropped_img = obj
                result.cx = 2*(x+w/2)/self.img_sm.shape[1]-1
                result.cy = -(2*(y+h/2)/self.img_sm.shape[0]-1)
                break
        return result

    def process(self):
        if self.OPENED_TYPE == self.SOURCE_TYPE['SEG_IMG']:
            return self.process_seg()

        result = BuoyReturn()

        kp, des = self.sift.detectAndCompute(self.img_gray, None)

        # BAD Bug fixing
        if len(kp) < 2:
            return result

        matches = self.flann.knnMatch(self.ori_des, des, k=2)

        points = []
        good_matches = []
        matchesMask = [[0, 0]]*len(matches)
        original = []
        fromCamera = []
        for i, (match_cam, match_ori) in enumerate(matches):
            if match_cam.distance < 0.8*match_ori.distance:
                matchesMask[i] = [1, 0]
                pt = kp[match_cam.trainIdx].pt
                points.append((int(pt[0]), int(pt[1])))
                good_matches.append(match_cam)
                original.append(self.ori_kp[match_cam.queryIdx].pt)
                fromCamera.append(kp[match_cam.trainIdx].pt)
        original = np.float32(original)
        fromCamera = np.float32(fromCamera)

        if len(good_matches) == 0:
            result.result_img = self.drawDebug(
                kp, matches, matchesMask)
            return result

        h, _ = cv2.findHomography(fromCamera, original, cv2.RANSAC, 3.0)
        inv_h, _ = cv2.findHomography(original, fromCamera, cv2.RANSAC, 3.0)

        if h is None:
            result.result_img = self.drawDebug(
                kp, matches, matchesMask)
            return result

        height, width = self.jiangshi.shape[:2]
        cropped = cv2.warpPerspective(self.img_sm, h, (width, height))
        trainBorder = np.float32(
            [[[0, 0], [0, height-1], [width-1, height-1], [width-1, 0]]])
        queryBorder = cv2.perspectiveTransform(trainBorder, inv_h)

        result.cropped_img = cropped

        # cv2.imshow('cropped', cropped)

        cx, cy = 0, 0

        for x, y in queryBorder[0]:
            cx += x
            cy += y
        cx = cx/4
        cy = cy/4
        # print(cx,cy)
        # print(self.img_sm.shape)
        cx /= self.img_sm.shape[1]
        cy /= self.img_sm.shape[0]
        cx *= 2
        cy *= 2
        cx -= 1
        cy -= 1
        cy *= -1

        if self.LASTFOUND is not None:
            time_diff = time.time()-(self.LASTFOUND.time)
            if time_diff < 2:
                distPc = self.calcDistPercent(
                    (cx, cy),
                    (self.LASTFOUND.cx, self.LASTFOUND.cy)
                )
                if distPc > 0.3:
                    result.result_img = self.drawDebug(
                        kp, matches, matchesMask, queryBorder=queryBorder)
                    return result

        gray_crop = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        _, th = cv2.threshold(
            gray_crop, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        circles = cv2.HoughCircles(
            th, cv2.HOUGH_GRADIENT, 1, 20,
            param1=50, param2=10, maxRadius=int(cropped.shape[0]/10))
        if circles is None:
            result.result_img = self.drawDebug(
                kp, matches, matchesMask, queryBorder=queryBorder)
            return result

        result.result_img = self.drawDebug(
            kp, matches, matchesMask, points, queryBorder)
        result.cx = cx
        result.cy = cy
        result.score = len(good_matches)/len(self.ori_kp)
        result.area = cv2.contourArea(queryBorder) / \
            self.img_gray.shape[0]/self.img_gray.shape[1]
        result.time = time.time()
        self.LASTFOUND = result
        return result

    def drawDebug(self, kp, matches, matchesMask, points=None, queryBorder=None):
        gray = self.img_gray.copy()
        if queryBorder is not None:
            cv2.polylines(gray, [np.int32(
                queryBorder)], True, 0, 5)
        if points is not None:
            for i, pt in enumerate(points):
                cv2.drawMarker(
                    gray, pt, 0, cv2.MARKER_STAR, 10, 10)

        draw_params = dict(matchColor=(0, 255, 0),
                           singlePointColor=(255, 0, 0),
                           matchesMask=matchesMask,
                           flags=0)
        flann_matches = cv2.drawMatchesKnn(
            self.jiangshi, self.ori_kp, gray, kp, matches, None, **draw_params)
        if points is not None:
            cv2.putText(flann_matches, "FOUND", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 7)
        else:
            cv2.putText(flann_matches, "NOT FOUND", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 7)
        flann_matches = cv2.resize(
            flann_matches, None, fx=1000/flann_matches.shape[1],
            fy=1000/flann_matches.shape[1])
        return flann_matches

    def calcDistPercent(self, pt, newpt):
        return (((newpt[0]-pt[0])**2+(newpt[1]-pt[1])**2)**0.5)/2
