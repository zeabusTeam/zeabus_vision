#!/usr/bin/python2.7
import rospy
import os
import cv2 as cv
import numpy as np
from time import time
from std_msgs.msg import Int64, Float64, Header, String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from zeabus_utility.msg import VisionQualification
from zeabus_utility.srv import VisionSrvQualification
from operator import itemgetter
from constant import AnsiCode
from vision_lib import OutputTools, ImageTools, TransformTools

image = ImageTools(sub_sampling=0.1)
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
        return VisionQualification()
    msg = VisionQualification()
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
    cv.imwrite('raw'+str(c)+'.png', image.display)
    # detector = cv.ORB_create()
    print(image.display.shape)
    detector = cv.xfeatures2d.SURF_create()
    filedir = os.path.dirname(os.path.abspath(__file__))
    heart_template = cv.imread(os.path.join(
        filedir, 'pictures/heart_canny.png'), 0)
    w, h = heart_template.shape[:2]
    '''
    heart = []
    for scale in np.linspace(0.3**0.5, 1.2**0.5, 10)[::-1]:
        heart.append(cv.resize(heart_template, (int(w/scale), int(h/scale))))
        heart.append(cv.resize(heart_template, (int(w), int(h/scale))))
        heart.append(cv.resize(heart_template, (int(w/scale), int(h))))
    '''
    # MIN_MATCH_COUNT = 10
    # FLANN_INDEX_KDITREE = 0
    # flannParam = dict(algorithm=FLANN_INDEX_KDITREE, tree=5)
    # flann = cv.FlannBasedMatcher(flannParam, {})
    # train_kp, train_des = detector.detectAndCompute(heart_template, None)
    # res = cv.drawKeypoints(heart_template.copy(), train_kp, None)
    edges = cv.Canny(image.bgr, 200, 206)
    th = 0.5
    drawed = False
    '''
    for each_heart in heart:
        if drawed:
            break
        res = cv.matchTemplate(edges, heart_template, cv.TM_CCOEFF_NORMED)
        print(res)
        w, h = each_heart.shape[:2]
        loc = np.where(res >= th)
        for pt in zip(*loc[::-1]):
            cv.rectangle(image.display, pt,
                         (pt[0]+w, pt[1]+h), (0, 255, 255), 2)
            drawed = True
            break
    '''
    output.publish(edges, 'gray', 'canny')
    output.publish(image.display, 'bgr', 'res')
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
        rospy.sleep(0.2)
        print(time()-a)
        c += 1
    # rospy.Service('vision/stake',
    #               VisionSrvQualification(), mission_callback)
    # output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    # output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
