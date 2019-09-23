#!/usr/bin/python2.7
"""
    File name: gen-data-vision-gate.py
    Author: AyumiizZ
    Date created: 2019/9/23
    Python Version: 2.7
"""
import rospy
from random import randint, randrange
from zeabus_vision.msg import TestVisionData, TestMpData

pub = rospy.Publisher('/vision/pub/gate', TestVisionData, queue_size=10)


class Data:
    def __init__(self):
        self.have_data = False
        while not self.have_data:
            print('--wait--')
            self.random_init()
            rospy.sleep(0.1)

    def random_init(self):
        luck = randint(0, 100)
        print('--luck--',luck)
        if luck >= 90:
            self.time = [rospy.Time.now()]
            self.have_data = True
            self.x = [randrange(-100, 100, _int=float)]
            self.y = [randrange(-100, 100, _int=float)]
            self.z = [randrange(-100, 100, _int=float)]
            pub.publish(message(self.x[-1], self.y[-1], self.z[-1]))

    def change(self):
        self.time.append(rospy.Time.now())
        self.x.append(self.x[-1] + randrange(-7, 7, _int=float))
        self.y.append(self.y[-1] + randrange(-7, 7, _int=float))
        self.z.append(self.z[-1] + randrange(-7, 7, _int=float))

    def feedback(self, msg):
        self.feed = list(msg.target)[:3]
        self.time.append(msg.header.stamp)
        self.x.append(self.x[-1] - self.feed[0]*randrange(0.8, 1.2, _int=float))
        self.y.append(self.y[-1] - self.feed[1]*randrange(0.8, 1.2, _int=float))
        self.z.append(self.z[-1] - self.feed[2]*randrange(0.8, 1.2, _int=float))

        # c = 0
        # print(self.time[-1],self.time[c])
        # while self.time[-1] - self.time[c] > 10:
        #     c += 1

        c = -10
        self.time = self.time[c:]
        self.x = self.x[c:]
        self.y = self.y[c:]
        self.z = self.z[c:]


def message(x=0, y=0, z=0, area=0):
    msg = TestVisionData()
    msg.area.data = float(area)
    msg.point1.x = x
    msg.point1.y = y
    msg.point1.z = z
    msg.header.stamp = rospy.Time.now()
    print('----Pub----')
    print(msg)
    return msg


if __name__ == "__main__":
    rospy.init_node('gen_data_vision_gate',anonymous=False)
    result = Data()
    while not rospy.is_shutdown():
        if result.have_data:
            result.change()
            pub.publish(message(x=result.x[-1], y=result.y[-1], z=result.z[-1]))
        else:
            pub.publish(message(area=-1))
        rospy.sleep(0.1)
    rospy.spin()
