#!/usr/bin/python
"""
    File name: gate
    Author: Shayennn
    Date created: 2019/04/20
    Python Version: 3.6
"""

import cv2 as cv

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from zeabus_utility.srv import VisionGate, VisionGateResponse
from gate_lib import Gate


SUB_SAMPLING = 0.3
PUBLIC_TOPIC = '/vision/mission/gate'
CAMERA_TOPIC = '/vision/front/image_rect_color/compressed'
DEBUG = {
    'console': True,
    'imageSource': True
}

process_obj = Gate()
image = None
last_found = [0.0, 0.0, 0.0, 0.0, 0.0]
bridge = CvBridge()


def imageCallback(msg):
    global image, bridge, SUB_SAMPLING
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        _log(str(e))
    image = cv.resize(cv_image, None, fx=SUB_SAMPLING, fy=SUB_SAMPLING)


def find_gate():
    global image, last_found
    output = None
    if image is not None:
        output = process_obj.doProcess(image)
        output = [float(i) for i in output]
    else:
        print('No input image')
    if output is not None:
        last_found = output
        return [1]+last_found
    else:
        return [0]+last_found


def gate_callback(msg):
    task = str(msg.task.data)
    req = str(msg.req.data)
    res = VisionGateResponse()
    if DEBUG['console'] or task == '':
        print('Service called', msg, req)
    if task in ['gate', '']:
        find_result = find_gate()
        res.found = find_result[0]
        res.cx1 = find_result[1]
        res.cy1 = find_result[2]
        res.x_left = find_result[3]
        res.x_right = find_result[4]
        res.area = find_result[5]
    else:
        if DEBUG['console'] or task == '':
            print('Not OK')
        res.found = 2
        res.cx1 = 0.0
        res.cy1 = 0.0
        res.x_left = 0.0
        res.x_right = 0.0
        res.area = 0.0
    return res


def main():
    rospy.init_node('vision_gate')
    if not rospy.is_shutdown():
        rospy.Service('gate_service', VisionGate(), gate_callback)
        rospy.Subscriber(CAMERA_TOPIC, CompressedImage, imageCallback)
        _log("Service is running.")
        rospy.spin()


def _log(msg, level='info'):
    module = 'Vision'
    submodule = 'Gate'
    real_msg = '[%s][%s] %s' % (module, submodule, msg)
    if level == 'error':
        rospy.logerr(real_msg)
    else:
        rospy.loginfo(real_msg)
    print(real_msg)


if __name__ == "__main__":
    main()
