import cv2 as cv
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from constant import AnsiCode


class OutputTools():
    def __init__(self):
        pass

    def log(self, msg, color=AnsiCode.DEFAULT):
        """
            print ('<----------') + str(msg) + ('---------->')
            #len of <---msg---> = 80
        """
        white_character = len(color) + 80
        temp = '<{:-^' + str(white_character) + '}>'
        print (temp.format(' ' + color +
                                 str(msg) + AnsiCode.DEFAULT + ' '))

    def img_is_none(self):
        print(AnsiCode.RED + 'img is none.'+'\n'
              'Please check topic name or check camera is running' +
              AnsiCode.DEFAULT)


class RosCmd():
    def __init__(self):
        self.bridge = CvBridge()

    def publish_result(self, img, type, topic_name):
        """
            publish picture
        """
        if img is None:
            img = np.zeros((200, 200))
            type = "gray"
        pub = rospy.Publisher(str(topic_name), Image, queue_size=10)
        if type == 'gray':
            msg = self.bridge.cv2_to_imgmsg(img, "mono8")
        elif type == 'bgr':
            msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        pub.publish(msg)
