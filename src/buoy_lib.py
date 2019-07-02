from __future__ import division
import cv2
import os
from sklearn.cluster import DBSCAN
import numpy as np


class VisionException(Exception):
    pass


class SourceIsNotOpened(VisionException):
    pass


class BuoyReturn:
    cx = 0.0
    cy = 0.0
    score = 0
    area = -1
    result_img = None


class Buoy:

    lockX = 750
    MIN_POINTS = 6

    SOURCE_TYPE = {
        'FILE': 0,
        'SINGLE_IMG': 1,
        'WEBCAM': 2
    }

    OPENED = False
    OPENED_TYPE = None

    def __init__(self):
        # Load Ref img
        filedir = os.path.dirname(os.path.abspath(__file__))
        jiangshi = cv2.imread(os.path.join(filedir, 'jiangshi.jpg'), 0)
        jiangshi = cv2.resize(jiangshi, None, fx=0.3, fy=0.3)
        self.jiangshi = cv2.medianBlur(jiangshi, 3)
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.ori_kp, self.ori_des = self.sift.detectAndCompute(jiangshi, None)

        # Init FLANN
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=100)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def openSource(self, sourceType, source=0):
        if sourceType == self.SOURCE_TYPE['FILE']:
            self.cv_dev = cv2.VideoCapture(source)
        if sourceType == self.SOURCE_TYPE['SINGLE_IMG']:
            self.img = source.copy()
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
        if self.OPENED_TYPE == self.SOURCE_TYPE['WEBCAM']:
            _, self.img = self.cv_dev.read()
        return _

    def preprocess(self):
        img = self.img.copy()
        zoom_rate = self.lockX/img.shape[1]
        self.img_sm = cv2.resize(img, None, fx=zoom_rate, fy=zoom_rate)
        self.img_sm = cv2.GaussianBlur(self.img_sm, (3, 3), 256)
        self.img_gray = cv2.cvtColor(self.img_sm, cv2.COLOR_BGR2GRAY)

    def process(self):
        result = BuoyReturn()

        kp, des = self.sift.detectAndCompute(self.img_gray, None)

        # BAD Bug fixing
        if len(kp) < 2:
            return result

        matches = self.flann.knnMatch(self.ori_des, des, k=2)

        points = []
        matchesMask = [[0, 0] for i in range(len(matches))]
        for i, (match_cam, match_ori) in enumerate(matches):
            if match_cam.distance < 0.8*match_ori.distance:
                matchesMask[i] = [1, 0]
                pt = kp[match_cam.trainIdx].pt
                points.append((int(pt[0]), int(pt[1])))

        labels = []

        if len(points) >= self.MIN_POINTS:
            clustering = DBSCAN(
                eps=int(self.img_gray.shape[0]/8),
                min_samples=self.MIN_POINTS)
            clustering.fit(points)
            labels = list(clustering.labels_)

        grouped = self.filterByLabels(points, labels)

        if len(grouped) == 0:
            result.result_img = self.drawDebug(
                kp, matches, labels, matchesMask)
            return result

        grouped = sorted(grouped, key=lambda (a, b): len(b), reverse=True)

        xs, ys = ([], [])
        for x, y in grouped[0][1]:
            xs.append(x)
            ys.append(y)

        ct = (sum(xs)/len(xs), sum(ys)/len(ys))

        rect = cv2.boundingRect(np.array(grouped[0][1]))

        result.result_img = self.drawDebug(
            kp, matches, labels, matchesMask, points, rect, ct)
        result.cx = 2*ct[0]/self.img_gray.shape[1]-1
        result.cy = 1-2*ct[1]/self.img_gray.shape[0]
        result.score = len(grouped[0][1])/len(points)
        result.area = rect[2]*rect[3]/self.img_gray.shape[0]/self.img_gray.shape[1]
        return result

    def filterByLabels(self, points, labels):
        labels = labels[:]
        u_labels = list(set(labels))
        if -1 in u_labels:
            u_labels.remove(-1)
        return [(label, [points[i] for i, lb in enumerate(labels) if lb == label])
                for label in u_labels]

    def drawDebug(self, kp, matches, labels, matchesMask, points=None, rect=None, ct=None):
        gray = self.img_gray.copy()
        if rect is not None:
            x, y, w, h = rect
            cv2.rectangle(gray, (x, y), (x+w, y+h), 0, 3)
        if ct is not None:
            ct = tuple([int(a) for a in ct])
            cv2.drawMarker(gray, ct, 0, cv2.MARKER_TILTED_CROSS, 50, 10)
        if points is not None:
            for i, pt in enumerate(points):
                if labels[i] != -1:
                    cv2.drawMarker(
                        gray, pt, 0, cv2.MARKER_TRIANGLE_DOWN, 30, 10)

        draw_params = dict(matchColor=(0, 255, 0),
                           singlePointColor=(255, 0, 0),
                           matchesMask=matchesMask,
                           flags=0)
        flann_matches = cv2.drawMatchesKnn(
            self.jiangshi, self.ori_kp, gray, kp, matches, None, **draw_params)
        if sum(labels) != -len(labels):
            cv2.putText(flann_matches, "FOUND", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 7)
        else:
            cv2.putText(flann_matches, "NOT FOUND", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 7)
        flann_matches = cv2.resize(
            flann_matches, None, fx=1000/flann_matches.shape[1],
            fy=1000/flann_matches.shape[1])
        return flann_matches
