#!/usr/bin/python
import cv2 as cv

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from zeabus_utility.srv import VisionBuoy, VisionBuoyResponse
from buoy_lib import Buoy
from vision_lib import ImageTools

from gate_buoy_debug_lib import gblog

USE_IMG_SEG = False
SUB_SAMPLING = 1
PUBLIC_TOPIC = '/vision/mission/buoy'
SEG_TOPIC = '/semantic_segmentation/compressed'
CAMERA_TOPIC = ImageTools().topic('front')
DEBUG = {
    'console': False,
    'oldbag': False
}
if DEBUG['oldbag']:
    CAMERA_TOPIC = '/stereo/right/image_rect_color/compressed'

process_obj = Buoy(rospy)
image = None
seg_img = None
result_pub = None
last_found = [0.0, 0.0, 0.0, 0.0, 0.0]
bridge = CvBridge()


def imageCallback(msg):
    global image, bridge, SUB_SAMPLING
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(msg)
    except CvBridgeError as e:
        _log(str(e), 'error')
    # cv.imshow('img', cv_image)
    # cv.waitKey(1)
    image = cv.resize(cv_image, None, fx=SUB_SAMPLING, fy=SUB_SAMPLING)


def segCallback(msg):
    global seg_img, bridge, SUB_SAMPLING
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(msg)
    except CvBridgeError as e:
        _log(str(e), 'error')
    # cv.imshow('seg', cv_image)
    # cv.waitKey(1)
    seg_img = cv.resize(cv_image, None, fx=SUB_SAMPLING, fy=SUB_SAMPLING)


def find_buoy():
    global image, seg_img, last_found, result_pub, bridge
    if image is not None:
        if USE_IMG_SEG:
            process_obj.openSource(
                process_obj.SOURCE_TYPE['SEG_IMG'], [image, seg_img])
        else:
            process_obj.openSource(
                process_obj.SOURCE_TYPE['SINGLE_IMG'], image)
        process_obj.read()
        process_obj.preprocess()
        res = process_obj.process()
        if res.result_img is not None:
            result_pub.publish(bridge.cv2_to_imgmsg(
                res.result_img, encoding="bgr8"))
        gblog(res)
    else:
        _log('No input image', 'error')
    if image is not None and res.score > 0:
        last_found = res
        return [1, res]
    else:
        return [0, res]


def buoy_callback(msg):
    task = str(msg.task.data)
    req = str(msg.req.data)
    res = VisionBuoyResponse()
    if DEBUG['console'] or task == '':
        print('Service called', task, req)
    if task in ['buoy', '']:
        find_result = find_buoy()
        res.found = find_result[0]
        res.cx = float(find_result[1].cx)
        res.cy = float(find_result[1].cy)
        res.area = float(find_result[1].area)
        res.score = float(find_result[1].score)
    else:
        if DEBUG['console'] or task == '':
            print('Not OK')
        res.found = -1
        res.cx = 0.0
        res.cy = 0.0
        res.area = 0.0
        res.score = 0.0
    return res


def main():
    global result_pub
    rospy.init_node('vision_buoy')
    if not rospy.is_shutdown():
        rospy.Service('/vision/buoy', VisionBuoy(), buoy_callback)
        result_pub = rospy.Publisher(
            PUBLIC_TOPIC+'/result', Image, queue_size=10)
        rospy.Subscriber(CAMERA_TOPIC, CompressedImage, imageCallback)
        rospy.Subscriber(SEG_TOPIC, CompressedImage, segCallback)
        _log("Service is running.")
        rospy.spin()


def _log(msg, level='info'):
    module = 'Vision'
    submodule = 'Buoy'
    real_msg = '[%s][%s] %s' % (module, submodule, msg)
    if level == 'error':
        rospy.logerr(real_msg)
    else:
        rospy.loginfo(real_msg)
    # print(real_msg)


if __name__ == "__main__":
    main()
