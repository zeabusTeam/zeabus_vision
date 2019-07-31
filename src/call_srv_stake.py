#!/usr/bin/python2.7
import rospy
from zeabus_utility.srv import VisionSrvStake
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node('call_service')
    service_name = 'vision/stake'
    print('wait service')
    rospy.wait_for_service(service_name)
    print('service start')
    call = rospy.ServiceProxy(service_name, VisionSrvStake)
    while not rospy.is_shutdown():
        try:
            print('calling')
            res = call(String('stake'),String('vampire'))
            print(res)
        except:
            pass
        rospy.sleep(0.05)