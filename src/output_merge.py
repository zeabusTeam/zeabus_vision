#!/usr/bin/python2.7
"""
    File name: output_merge.py
    Author: KNW
    Date created: 2019/09/23
    Python Version: 2.7
"""
import rospy
from vision_lib import OutputTools, AnsiCode
from zeabus_vision.msg import TestVisionData, VisionMergeData


pub = rospy.Publisher('/vision/pub/all_task', VisionMergeData, queue_size=3)

output = OutputTools()

class Submessage():
    def __init__(self, default_msg, task, queue_size=3):
        self.msg = default_msg()
        self.queue_size = queue_size
        rospy.Subscriber("/vision/pub/"+task, default_msg,
                         callback=self.callback, queue_size=self.queue_size)

    def callback(self, msg):
        self.msg = msg
        # output.log('Callback', color=AnsiCode.YELLOW)

def message(gate, flare, drum, golf):
    msg = VisionMergeData()
    msg.gate = gate.msg
    msg.flare = flare.msg
    msg.drum = drum.msg
    msg.golf = golf.msg
    output.log('Publishing', color=AnsiCode.GREEN)
    print(msg)
    return msg


if __name__ == "__main__":
    rospy.init_node('gen_data_mp')
    gate = Submessage(TestVisionData,task='gate')
    flare = Submessage(TestVisionData, task='flare')
    drum = Submessage(TestVisionData, task='drum')
    golf = Submessage(TestVisionData, task='golf')
    while not rospy.is_shutdown():
        pub.publish(message(gate, flare, drum, golf))
        rospy.sleep(0.1)
    rospy.spin()
