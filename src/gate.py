#!/usr/bin/python
import cv2 as cv

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from zeabus_utility.srv import VisionGate, VisionGateResponse
from gate_lib import Gate
from vision_lib import ImageTools

from gate_buoy_debug_lib import gblog


SUB_SAMPLING = 1
PUBLIC_TOPIC = '/vision/mission/gate'
CAMERA_TOPIC = ImageTools().topic('front')
DEBUG = {
    'console': False,
    'oldbag': False
}
if DEBUG['oldbag']:
    CAMERA_TOPIC = '/stereo/right/image_rect_color/compressed'

process_obj = Gate()
image = None
result_pub = None
mask_pub = None
last_found = [0.0, 0.0, 0.0, 0.0, 0.0]
bridge = CvBridge()


def imageCallback(msg):
    global image, bridge, SUB_SAMPLING
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(msg)
    except CvBridgeError as e:
        _log(str(e), 'error')
    image = cv.resize(cv_image, None, fx=SUB_SAMPLING, fy=SUB_SAMPLING)


def find_gate():
    global image, last_found, result_pub, mask_pub, bridge
    output = None
    if image is not None:
        output, img, mask = process_obj.doProcess(image)
        result_pub.publish(bridge.cv2_to_imgmsg(img, encoding="bgr8"))
        mask_pub.publish(bridge.cv2_to_imgmsg(mask))
        if output is not None:
            output = [float(i) for i in output]
    else:
        _log('No input image', 'error')
    if image is not None and output is not None:
        last_found = output
        return [1]+last_found
    else:
        return [0]+last_found


def gate_callback(msg):
    task = str(msg.task.data)
    req = str(msg.req.data)
    res = VisionGateResponse()
    if DEBUG['console'] or task == '':
        print('Service called', task, req)
    if req == 'ml':
        process_obj.useml = True
    if req == 'if':
        process_obj.useml = False
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
    # gblog(res.__dict__)
    return res


def main():
    global result_pub, mask_pub
    rospy.init_node('vision_gate')
    if not rospy.is_shutdown():
        rospy.Service('/vision/gate', VisionGate(), gate_callback)
        result_pub = rospy.Publisher(
            PUBLIC_TOPIC+'/result', Image, queue_size=10)
        mask_pub = rospy.Publisher(
            PUBLIC_TOPIC+'/mask', Image, queue_size=10)
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
    # print(real_msg)


if __name__ == "__main__":
    main()
