#!/usr/bin/python2.7
import rospy
from zeabus_utility.srv import VisionSrvCoffin, VisionSrvPath
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node('call_service_coffin')
    service_name = 'vision/coffin'
    print('wait service')
    rospy.wait_for_service(service_name)
    print('service start')
    call = rospy.ServiceProxy(service_name, VisionSrvCoffin)
    while not rospy.is_shutdown():
        try:
            print('calling')
            res = call(String('exposed'), String('coffin'))
            print(res)
        except:
            pass
        rospy.sleep(0.05)
