#!/usr/bin/python2.7

import rospy
from zeabus_utility.srv import SendControlCommand, SendControlCommandResponse


def mission_callback(msg):
    print(msg)
    return SendControlCommandResponse()


if __name__ == "__main__":
    rospy.init_node('test_srv_node')
    rospy.Service('/control/thruster', SendControlCommand, mission_callback)
    rospy.spin()