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
from zeabus_vision.srv import TestSrvVisionData

class Submessage():
    def __init__(self, default_msg, task, queue_size=3):
        self.msg = default_msg()
        self.queue_size = queue_size
        rospy.Subscriber("/vision/pub/"+task, default_msg,
                         callback=self.callback, queue_size=self.queue_size)

    def callback(self, msg):
        self.msg = msg
        # output.log('Callback', color=AnsiCode.YELLOW)

all_task = Submessage(VisionMergeData,'all_task')
        
def gate_callback(msg):
    """
        When call service it will run this 
        Returns:
            a group of process value from this program
    """
    task = str(msg.task)
    request = str(msg.request)
    if task == 'gate':
        print(all_task.msg.gate)
        return all_task.msg.gate
    
def flare_callback(msg):
    task = str(msg.task)
    request = str(msg.request)
    if task == 'flare':
        return all_task.msg.flare

def drum_callback(msg):
    task = str(msg.task)
    request = str(msg.request)
    if task == 'drum':
        return all_task.msg.drum

def golf_callback(msg):
    task = str(msg.task)
    request = str(msg.request)
    if task == 'golf':
        return all_task.msg.golf



def mission_callback(msg):
    pass

if __name__ == '__main__':
    rospy.init_node('vision_service', anonymous=False)
    rospy.Subscriber('/vision/pub/all_task', VisionMergeData, all_task.callback)
    rospy.Service('vision/srv/gate',
                  TestSrvVisionData(), gate_callback)
    rospy.Service('vision/srv/flare',
                  TestSrvVisionData(), flare_callback)
    rospy.Service('vision/srv/drum',
                  TestSrvVisionData(), drum_callback)
    rospy.Service('vision/srv/golf',
                  TestSrvVisionData(), golf_callback)
    rospy.spin()