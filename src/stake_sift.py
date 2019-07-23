#!/usr/bin/python2.7
import rospy
import os
import cv2 as cv
import numpy as np
from time import time
from std_msgs.msg import Int64, Float64, Header, String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from zeabus_utility.msg import VisionBox
from zeabus_utility.srv import VisionSrvStake, VisionSrvStakeResponse
from operator import itemgetter
from constant import AnsiCode
from vision_lib import OutputTools, ImageTools, TransformTools

image = ImageTools(sub_sampling=0.3)
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
    task = str(msg.task.data)
    request = str(msg.request.data)
    print('task',task,'req',request)
    if task == 'stake' and request == 'vampire':
        return find_vampire()
    elif task == 'stake' and request == 'heart':
        return find_heart()
    elif task == 'stake' and request in ['left', 'right']:
        return find_hole(request)


def q_area(box, n=4):
    area = 0.0
    j = n-1
    for i in range(0, n):
        area += (box[j][0] + box[i][0]) * (box[j][1] - box[i][1])
        j = i
    print(area)
    return abs(area / 2.0)


def to_box(state=0, box=0, color=(0, 255, 0), area=0.0, center=True):
    shape = image.display.shape[:2]
    sort = sorted(box, key=lambda x: x[0])
    bottom = sort[:2]
    bottom = sorted(bottom, key=lambda x: x[1])
    pt4 = bottom[0]
    pt1 = bottom[1]
    top = sort[2:]
    top = sorted(top, key=lambda x: x[1])
    pt3 = top[0]
    pt2 = top[1]
    msg = VisionBox()
    msg.state = state
    print(shape[0]*shape[1], 'shape')
    if area == -1:
        msg.area = q_area(box)/(shape[0]*shape[1])
    elif area > 0:
        msg.area = area/(shape[0]*shape[1])
    for i in range(1, 5):
        print('pt'+str(i), tuple(eval('pt'+str(i))))
        cv.putText(image.display, str(i), tuple(eval('pt'+str(i))),
                   cv.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv.LINE_AA)
    if center:
        cx = cy = 0
        for pt in box:
            cx += pt[0]
            cy += pt[1]
        cx /= 4
        cy /= 4
        cv.putText(image.display, 'c', (cx, cy),
                   cv.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv.LINE_AA)
        cv.circle(image.display, (cx, cy), 5, color, -1)
        msg.point_1 = transform.convert_to_point((cx, cy), shape)
        print('msg', msg)
    else:
        msg.point_1 = transform.convert_to_point(pt1, shape)
        msg.point_2 = transform.convert_to_point(pt2, shape)
        msg.point_3 = transform.convert_to_point(pt3, shape)
        msg.point_4 = transform.convert_to_point(pt4, shape)

    return msg


def message(state=0, box=0, area=0.0, center=True):
    response = VisionSrvStakeResponse()
    if state < 0:
        return response
    if state >= 1:
        response.data = to_box(state=state, box=box, area=area, center=center)
    output.log('pubing', AnsiCode.GREEN)
    output.publish(image.display, 'bgr', 'display')
    print(response)
    return response


def find_vampire(c=0):
    if image.bgr is None:
        output.img_is_none()
        return message(state=-1)
    image.renew_display()
    # cv.imwrite('raw'+str(c)+'.png',image.display)
    # detector = cv.ORB_create()
    detector = cv.xfeatures2d.SIFT_create()
    filedir = os.path.dirname(os.path.abspath(__file__))
    heart_template = cv.imread(os.path.join(
        filedir, 'pictures/full0.3.png'), 0)
    MIN_MATCH_COUNT = 20
    FLANN_INDEX_KDITREE = 0
    flannParam = dict(algorithm=FLANN_INDEX_KDITREE, tree=5)
    flann = cv.FlannBasedMatcher(flannParam, {})
    # cv.imwrite('big-raw' + str(c) + '.png', image.display)
    train_kp, train_des = detector.detectAndCompute(heart_template, None)
    print(len(train_kp))
    res = cv.drawKeypoints(heart_template.copy(), train_kp, None)
    output.publish(res, 'bgr', 'res')
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
    himg, wimg = image.display.shape[:2]
    cv.putText(image.display, str(len(goodMatch))+"/"+str(len(train_kp)), (0, himg),
               cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA)
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
        print('q', queryBorder[0])
        print('qnp', [np.int32(queryBorder[0])])
        cv.polylines(image.display, [np.int32(
            queryBorder)], True, (0, 255, 0), 5)
        return message(state=1, box=np.int64(queryBorder[0]), area=-1, center=False)
    else:
        print "Not Enough match found- %d/%d" % (
            len(goodMatch), MIN_MATCH_COUNT)
        return message()
    output.publish(edges, 'gray', 'canny')


def get_mask():
    image.to_hsv()
    upper = np.array([72, 255, 255], dtype=np.uint8)
    lower = np.array([28, 0, 0], dtype=np.uint8)
    mask = cv.inRange(image.hsv, lower, upper)
    return mask


def get_template():
    filedir = os.path.dirname(os.path.abspath(__file__))
    heart_template = cv.imread(os.path.join(
        filedir, 'pictures/med.png'), 3)
    upper = np.array([72, 255, 255], dtype=np.uint8)
    lower = np.array([28, 0, 0], dtype=np.uint8)
    mask = cv.inRange(heart_template, lower, upper)
    return mask


def find_heart(c=0):
    if image.bgr is None:
        output.img_is_none()
        return message(state=-1)
    image.renew_display()
    mask = get_mask()
    output.publish(mask, 'gray', 'mask')
    contours = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    kernel_vertical = image.get_kernel(ksize=(25, 25))
    res = []
    for cnt in contours:
        if cv.contourArea(cnt) < 500:
            continue
        rect = cv.minAreaRect(cnt)
        print(rect)
        box1 = cv.boxPoints(rect)
        box1 = np.int64(box1)
        cv.drawContours(image.display, [box1], 0, (0, 0, 255), 2)
    output.publish(image.display, 'bgr', 'heart')


def find_hole(request):
    if image.bgr is None:
        output.img_is_none()
        return message(state=-1)
    image.renew_display()
    mask = get_mask()
    output.publish(mask, 'gray', 'mask')
    kernel_vertical = image.get_kernel(ksize=(25, 25))
    vertical = cv.erode(mask.copy(), kernel_vertical)
    contours = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    res = []
    for cnt in contours:
        if cv.contourArea(cnt) < 500:
            continue
        rect = cv.minAreaRect(cnt)
        ratio = (rect[1][1]/rect[1][0])*5
        if ratio < 7 or ratio > 9:
            continue
        print((rect[1][1]/rect[1][0])*5)
        box1 = cv.boxPoints(rect)
        box1 = np.int64(box1)
        cv.drawContours(image.display, [box1], 0, (0, 0, 255), 2)
        res.append(rect)
    if res == []:
        output.publish(image.display, 'bgr', 'heart')
        return message()
    print(res)
    res = sorted(res, key=lambda x: x[0][0], reverse=request == 'right')
    box_data = cv.boxPoints(res[0])
    box = np.int64(box_data)
    cv.drawContours(image.display, [box], 0, (0, 255, 0), 2)
    output.publish(image.display, 'bgr', 'heart')
    print(box_data[1][0],box_data[1][1])
    return message(state=1,box=box, area=box_data[1][0]*box_data[1][1], center=True)


def nothing(x):
    pass


if __name__ == '__main__':
    rospy.init_node('vision_stake', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber(image.topic('front'), CompressedImage, image.callback)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    # while not rospy.is_shutdown():
    #     find_hole('right')
    #     print(1)
    rospy.Service('vision/stake',
                  VisionSrvStake(), mission_callback)
    output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
