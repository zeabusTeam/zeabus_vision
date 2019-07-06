#!/usr/bin/python2.7
import rospy
import os
import cv2 as cv
import numpy as np
from time import time
from std_msgs.msg import Int64, Float64, Header, String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from zeabus_utility.msg import VisionStake
from zeabus_utility.srv import VisionSrvStake
from operator import itemgetter
from constant import AnsiCode
from vision_lib import OutputTools, ImageTools, TransformTools

image = ImageTools(sub_sampling=0.5)
output = OutputTools(topic='/vision/stake/')
transform = TransformTools()
seq = 1


class Log:
    def __init__(self):
        self.time = time()

    def reset_all(self):
        pass


def mission_callback(msg):
    """
        When call service it will run this 
        Returns:
            a group of process value from this program
    """
    if image.bgr is None:
        output.img_is_none()
        return message(state=-1)
    task = str(msg.task.data)
    request = str(msg.request.data)
    if task == 'stake' and request == 'heart':
        return find_heart()
    elif task == 'stake' and request == 'hole':
        return find_hole()


def message(state=0, cx1=0.0, cy1=0.0, cx2=0.0, cy2=0.0, area=0.0,
            name='stake'):
    global seq
    if state < 0:
        return VisionStake()
    msg = VisionStake()
    header = Header()
    header.stamp = rospy.Time.now()
    header.seq = seq
    seq += 1
    msg.header = header
    msg.type = Int64(state)
    msg.name = String(name)
    msg.point1 = transform.to_point(cx1, cy1, image.display.shape[:2])
    msg.point2 = transform.to_point(cx2, cy2, image.display.shape[:2])
    msg.area = Float64(area)
    print(msg)
    return msg

    # filedir = os.path.dirname(os.path.abspath(__file__))
    # jiangshi = cv2.imread(os.path.join(filedir, 'jiangshi.jpg'), 0)


def find_heart(c=0):
    if image.bgr is None:
        output.img_is_none()
        return message(state=-1)
    image.renew_display()
    # detector = cv.ORB_create()
    detector = cv.xfeatures2d.SIFT_create()
    filedir = os.path.dirname(os.path.abspath(__file__))
    heart_template = cv.imread(os.path.join(
        filedir, 'pictures/big-raw.png'), 0)
    MIN_MATCH_COUNT = 15
    FLANN_INDEX_KDITREE = 0
    flannParam = dict(algorithm=FLANN_INDEX_KDITREE, tree=5)
    flann = cv.FlannBasedMatcher(flannParam, {})
    # cv.imwrite('big-raw' + str(c) + '.png', image.display)
    train_kp, train_des = detector.detectAndCompute(heart_template, None)
    res = cv.drawKeypoints(heart_template.copy(), train_kp, None)
    edges = cv.Canny(image.bgr, 200, 206)
    queryKP, queryDesc = detector.detectAndCompute(image.display, None)
    matches = flann.knnMatch(queryDesc, train_des, k=2)
    res2 = cv.drawKeypoints(image.display.copy(), queryKP, None)
    output.publish(res2, 'bgr', 'res2')
    # cv.imshow('qkp', res2)
    goodMatch = []
    for m, n in matches:
        if(m.distance < 0.75*n.distance):
            goodMatch.append(m)
    if(len(goodMatch) > MIN_MATCH_COUNT):
        tp = []
        qp = []
        for m in goodMatch:
            tp.append(train_kp[m.trainIdx].pt)
            qp.append(queryKP[m.queryIdx].pt)
        tp, qp = np.float32((tp, qp))
        H, status = cv.findHomography(tp, qp, cv.RANSAC, 3.0)
        h, w = heart_template.shape[:2]
        trainBorder = np.float32([[[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]])
        queryBorder = cv.perspectiveTransform(trainBorder, H)
        cv.polylines(image.display, [np.int32(
            queryBorder)], True, (0, 255, 0), 5)
    else:
        print "Not Enough match found- %d/%d" % (
            len(goodMatch), MIN_MATCH_COUNT)
    output.publish(edges, 'gray', 'canny')
    output.publish(res, 'bgr', 'res')
    output.publish(image.display, 'bgr', 'display')
    return message()


def find_hole():
    pass


def nothing(x):
    pass


if __name__ == '__main__':
    rospy.init_node('vision_stake', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber(image.topic('front'), CompressedImage, image.callback)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    # cv.namedWindow('frame')
    # cv.createTrackbar('lower', 'frame', 0, 255, nothing)
    c = 0
    while not rospy.is_shutdown():
        a = time()
        find_heart(c)
        c += 1
        print('time',time()-a)
        # rospy.sleep(0.1)
    # rospy.Service('vision/stake',
    #               VisionSrvStake(), mission_callback)
    # output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    # output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
