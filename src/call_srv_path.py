#!/usr/bin/python2.7
import rospy
from zeabus_utility.srv import VisionSrvPath
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node('call_service')
    service_name = 'vision/path'
    print('wait service')
    rospy.wait_for_service(service_name)
    print('service start')
    call = rospy.ServiceProxy(service_name, VisionSrvPath)
    last = 0
    i = 0
    while not rospy.is_shutdown():
        try:
            res = call(String('path'), String('path'))
            if last is not -1 and res.data.state is -1:
                print('Image is none...')
            elif res.data.state is -2:
                print('Size bug')
            elif res.data.state >= 0:
                i += 1
                print('Calling {} times'.format(i))
            last = res.data.state
        except:
            if i is not 0:
                print('wait service')
            i = 0
        rospy.sleep(0.05)
