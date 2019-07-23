from __future__ import division
import cv2
import os
from sklearn.cluster import DBSCAN
import numpy as np
import time


class VisionException(Exception):
    pass


class SourceIsNotOpened(VisionException):
    pass


class BuoyReturn:
    time = 0.0
    cx = 0.0
    cy = 0.0
    score = 0
    area = -1
    result_img = None


class Buoy:

    lockX = 750
    MIN_POINTS = 5

    SOURCE_TYPE = {
        'FILE': 0,
        'SINGLE_IMG': 1,
        'WEBCAM': 2
    }

    LASTFOUND = None

    OPENED = False
    OPENED_TYPE = None

    def __init__(self):
        # Load ML Lib
        # self.ML = BuoyML()

        # Load Ref img
        filedir = os.path.dirname(os.path.abspath(__file__))
        jiangshi = cv2.imread(os.path.join(filedir, 'jiangshi.jpg'), 0)
        self.jiangshi = cv2.resize(jiangshi, None, fx=0.1, fy=0.1)
        # self.jiangshi = cv2.medianBlur(self.jiangshi, 7)
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.ori_kp, self.ori_des = self.sift.detectAndCompute(self.jiangshi, None)

        # Init FLANN
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        # self.flann = cv2.BFMatcher()

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
        # img_yuv = cv2.cvtColor(self.img_sm, cv2.COLOR_BGR2YUV)
        # img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])
        # self.img_sm = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
        # self.img_sm = cv2.GaussianBlur(self.img_sm, (3, 3), 256)
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

        grouped = sorted(grouped, key=lambda (a, b): len(b), reverse=True)
        newGroup = []
        for i, dat in enumerate(grouped):
            rect = cv2.boundingRect(np.array(dat[1]))
            if rect[2]*rect[3]/self.img_gray.shape[0]/self.img_gray.shape[1] > 4e-04:
                newGroup.append(dat)
        grouped = newGroup

        if len(grouped) == 0:
            result.result_img = self.drawDebug(
                kp, matches, labels, matchesMask)
            return result

        xs, ys = ([], [])
        for x, y in grouped[0][1]:
            xs.append(x)
            ys.append(y)

        ct = (sum(xs)/len(xs), sum(ys)/len(ys))

        rect = cv2.boundingRect(np.array(grouped[0][1]))

        if rect[2]/rect[3] > 4 or rect[3]/rect[2] > 4:
            result.result_img = self.drawDebug(
                kp, matches, labels, matchesMask)
            return result

        cx = 2*ct[0]/self.img_gray.shape[1]-1
        cy = 1-2*ct[1]/self.img_gray.shape[0]

        if self.LASTFOUND is not None:
            time_diff = time.time()-(self.LASTFOUND.time)
            if time_diff < 2:
                distPc = self.calcDistPercent(
                    (cx, cy),
                    (self.LASTFOUND.cx, self.LASTFOUND.cy)
                )
                # print(distPc)
                if distPc > 0.3:
                    result.result_img = self.drawDebug(
                        kp, matches, labels, matchesMask)
                    return result

        cropped = self.cropRect(self.img_sm, rect)
        gray_crop = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        _, th = cv2.threshold(
            gray_crop, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        circles = cv2.HoughCircles(
            th, cv2.HOUGH_GRADIENT, 1, 20,
            param1=50, param2=10, maxRadius=int(cropped.shape[0]/10))
        if circles is None:
            result.result_img = self.drawDebug(
                kp, matches, labels, matchesMask)
            return result
        # if circles is not None:
        #     circles = np.round(circles[0, :]).astype("int")
        #     for (x, y, r) in circles:
        #         cv2.circle(cropped, (x, y), r, (0, 255, 0), 4)
        #         cv2.rectangle(cropped, (x - 5, y - 5),
        #                       (x + 5, y + 5), (0, 128, 255), -1)
        # cv2.imshow('isBuoy?', cropped)
        # cv2.imshow('isBuoy2?', th)
        # if cv2.waitKey(0) == ord('y'):
        #     self.save(cropped, 'buoy_train/p/')
        # else:
        #     self.save(cropped, 'buoy_train/n/')

        # print (self.ML.getprop(cropped))
        # if self.ML.predict(cropped) == 0:
        #     result.result_img = self.drawDebug(
        #         kp, matches, labels, matchesMask)
        #     return result

        result.result_img = self.drawDebug(
            kp, matches, labels, matchesMask, points, rect, ct)
        result.cx = cx
        result.cy = cy
        result.score = len(grouped[0][1])/len(points)
        result.area = rect[2]*rect[3] / \
            self.img_gray.shape[0]/self.img_gray.shape[1]
        result.time = time.time()
        self.LASTFOUND = result
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
        # if sum(labels) != -len(labels):
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

    def cropRect(self, img, rect):
        x, y, w, h = rect
        if x-40 > 0:
            x -= 40
        else:
            x = 0
        if y-40 > 0:
            y -= 40
        else:
            y = 0
        if x+w+40 < img.shape[1]:
            w += 40
        if y+h+40 < img.shape[0]:
            h += 40
        return img[y:y+h, x:x+w]

    def calcDistPercent(self, pt, newpt):
        return (((newpt[0]-pt[0])**2+(newpt[1]-pt[1])**2)**0.5)/2

    def save(self, img, path):
        cv2.imwrite(path+str(int(time.time()*1000000))+'.png', img)