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
from vision_lib import OutputTools, ImageTools, TransformTools, Detector


image = ImageTools(sub_sampling=0.5)
output = OutputTools(topic='/vision/stake/')
transform = TransformTools()
detector = Detector(picture_name='stake-full-0.3-usa.png', min_match=25)
seq = 1


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
    print('task', task, 'req', request)
    if task == 'stake' and request == 'vampire':
        return find_vampire()
    elif task == 'stake' and request == 'heart':
        return find_heart()
    elif task == 'stake' and request in ['left', 'right']:
        return find_hole(request)




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
        for i in range(1, 5):
            print('pt'+str(i), tuple(eval('pt'+str(i))))
            cv.putText(image.display, str(i), tuple(eval('pt'+str(i))),
                       cv.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv.LINE_AA)
        msg.point_1 = transform.convert_to_point(pt1, shape)
        msg.point_2 = transform.convert_to_point(pt2, shape)
        msg.point_3 = transform.convert_to_point(pt3, shape)
        msg.point_4 = transform.convert_to_point(pt4, shape)

    return msg


def message(state=0, box=0, area=0.0, center=True):
    response = VisionSrvStakeResponse()
    if state < 0:
        return response
    elif state >= 1:
        response.data = to_box(state=state, box=box,
                               area=area, center=center)
    output.log('Publishing display', AnsiCode.GREEN)
    markpoints()
    output.publish(image.display, 'bgr', 'display')
    print(response)
    return response


def markpoints():
    cv.rectangle(image.display, (102, 160), (188, 246), (255, 0, 0), 2)
    cv.circle(image.display, (145, 203), 5, (0, 0, 255), 1)
    cv.circle(image.display, (145, 203), 10, (0, 0, 255), 1)
    cv.line(image.display, (130, 203), (160, 203), (0, 0, 255), 1)
    cv.line(image.display, (145, 188), (145, 218), (0, 0, 255), 1)


def find_vampire():
    if image.bgr is None:
        output.img_is_none()
        return message(state=-1)
    image.renew_display()

    query_keypoint, query_desc = detector.compute(image.display)
    matches = detector.flann.knnMatch(query_desc, detector.train_desc, k=2)

    good_match = []
    for m, n in matches:
        if(m.distance < 0.80*n.distance):
            good_match.append(m)
    himg, wimg = image.display.shape[:2]

    # print
    cv.putText(image.display, 'Vampire',
               (3, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255),
               2, cv.LINE_AA)

    if(len(good_match) > detector.MIN_MATCH_COUNT):
        tp = []
        qp = []
        for m in good_match:
            tp.append(detector.train_keypoint[m.trainIdx].pt)
            qp.append(query_keypoint[m.queryIdx].pt)
        tp, qp = np.float32((tp, qp))
        H, status = cv.findHomography(tp, qp, cv.RANSAC, 3.0)
        query_border = cv.perspectiveTransform(detector.get_train_border(), H)
        cv.polylines(image.display, [np.int32(
            query_border)], True, (0, 255, 0), 5)
        box = np.int64(query_border[0])

        # print #/89 in display
        detect_point = str(len(good_match))+"/" + \
            str(len(detector.train_keypoint))
        cv.putText(image.display, detect_point,
                   (3, himg-10), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0),
                   2, cv.LINE_AA)
        output.log('FOUND', AnsiCode.GREEN)
        return message(state=1, box=box, area=-1, center=False)
    else:
        # print #/89 in display
        detect_point = str(len(good_match))+"/" + \
            str(len(detector.train_keypoint))
        cv.putText(image.display, detect_point,
                   (3, himg-10), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255),
                   2, cv.LINE_AA)
        output.log('NOT FOUND', AnsiCode.RED)
        return message()


def get_mask():
    image.to_hsv()
    # upper = np.array([72, 255, 255], dtype=np.uint8)
    # lower = np.array([28, 0, 0], dtype=np.uint8)
    upper = np.array([57, 255, 243], dtype=np.uint8)
    lower = np.array([19, 87, 0], dtype=np.uint8)
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


def find_heart():
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
    cv.putText(image.display, 'heart',
               (2, 24), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255),
               2, cv.LINE_AA)
    res = []
    for cnt in contours:
        if cv.contourArea(cnt) < 500:
            continue
        rect = ((x, y), (w, h), angle) = cv.minAreaRect(cnt)
        print('angle', angle)
        if angle > -22.5 or angle < -67.5:
            continue
        box = cv.boxPoints(rect)
        box = np.int64(box)
        cv.drawContours(image.display, [box], 0, (0, 0, 255), 2)
        res.append(rect)
    if res == []:
        return message()
    res = sorted(res, key=lambda x: x[1][0]*x[1][1], reverse=True)
    box_data = cv.boxPoints(res[0])
    box = np.int64(box_data)
    cv.drawContours(image.display, [box], 0, (0, 255, 0), 2)
    print(box_data[1][0], box_data[1][1])
    area = box_data[1][0]*box_data[1][1]
    return message(state=1, box=box, area=area, center=True)


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
    cv.putText(image.display, request,
               (2, 24), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255),
               2, cv.LINE_AA)
    res = []
    for cnt in contours:
        if cv.contourArea(cnt) < 500:
            continue

        rect = ((x, y), (w, h), angle) = cv.minAreaRect(cnt)

        if angle < -22.5 and angle > -67.5:
            continue

        x, y, w, h = cv.boundingRect(cnt)
        roi = mask[y:y + h, x:x + w]
        if w > h:
            continue

        ratio = (1.0*h/w)*5
        if ratio < 6.5 or ratio > 9.5:
            continue
        print('ratio', ratio)

        box = cv.boxPoints(rect)
        box = np.int64(box)
        cv.drawContours(image.display, [box], 0, (0, 0, 255), 2)

        print('area_ratio', (1.0*w*h)/cv.countNonZero(roi),
              x, y, w, h, cv.countNonZero(roi))
        if ((1.0*w*h)/cv.countNonZero(roi) < 2.7):
            continue

        print('wh')
        box = cv.boxPoints(rect)
        box = np.int64(box)
        cv.drawContours(image.display, [box], 0, (0, 255, 255), 2)
        res.append(rect)
    if res == []:
        return message()
    print(res)
    res = sorted(res, key=lambda x: x[0][0], reverse=(request == 'right'))
    box_data = cv.boxPoints(res[0])
    box = np.int64(box_data)
    cv.drawContours(image.display, [box], 0, (0, 255, 0), 2)
    print(box_data[1][0], box_data[1][1])
    area = box_data[1][0]*box_data[1][1]
    return message(state=1, box=box, area=area, center=True)


if __name__ == '__main__':
    rospy.init_node('vision_stake', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber(image.topic('front'), CompressedImage, image.callback)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    rospy.Service('vision/stake',
                  VisionSrvStake(), mission_callback)
    output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
