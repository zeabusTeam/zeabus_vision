import cv2 as cv
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from constant import AnsiCode


class OutputTools:
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


class RosCmd:
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


class ImageTools:
    def __init__(self, sub_sampling=0.3):
        self.bgr = None
        self.display = None
        self.sub_sampling = sub_sampling

    def to_hsv(self):
        self.hsv = cv.cvtColor(self.bgr, cv.COLOR_BGR2HSV)

    def to_gray(self):
        self.gray = cv.cvtColor(self.bgr, cv.COLOR_BGR2GRAY)

    def to_ycrcb(self):
        self.ycrcb = cv.cvtColor(self.bgr, cv.COLOR_BGR2YCrCb)

    def to_hls(self):
        self.hls = cv.cvtColor(self.bgr, cv.COLOR_BGR2HLS)

    def callback(self, msg):
        arr = np.fromstring(msg.data, np.uint8)
        self.bgr = cv.resize(cv.imdecode(arr, 1), (0, 0),
                             fx=self.sub_sampling, fy=self.sub_sampling)
        self.display = self.bgr.copy()

    def topic(self, camera):
        if camera == 'front':
            return '/vision/front/image_rect_color/compressed'
        elif camera == 'bottom':
            return '/vision/bottom/image_raw/compressed'

    def renew_display(self):
        self.display = self.bgr.copy()


class Statistics:
    def __init__(self):
        pass

    def convert_to_oneD(self, data):
        if len(data.shape) == 2:
            return data.ravel()
        return data

    def convert_to_np(self, data):
        return np.array(data)

    def get_mode(self, data):
        data = self.convert_to_oneD(data)
        count = np.bincount(data)
        max = count.max()
        count = list(count)
        return count.index(max)

    def get_median(self, data):
        return np.median(data)

    def get_mean(self, data):
        return data.mean()

    def get_std(self, data):
        return data.std()

    def get_max(self, data):
        return data.max()

    def get_min(self, data):
        return data.min()

    def get_range(self, data):
        return self.get_max(data) - self.get_min(data)

    def get_quantile(self, data, q):
        data = self.convert_to_np(data)
        return np.quantile(data, q / 4.)

    def get_percentile(self, data, q):
        data = self.convert_to_np(data)
        return np.percentile(data, q)

    def get_skewness(self, data, output_screen="True"):
        data = self.convert_to_oneD(data)
        min = self.get_min(data)
        max = self.get_max(data)
        std = self.get_std(data)
        mode = self.get_mode(data)
        mean = self.get_mean(data)
        median = self.get_median(data)

        skewness = ""

        if mean < median < mode:
            skewness = "Negative direction"
        elif mode < median < mean:
            skewness = "Positive direction"
        elif mode == median == mean:
            skewness = "No skew"
        else:
            skewness = "Don't know"

        if output_screen:
            print("MODE:", mode, "MED:", median, "MEAN:", mean)
            print("STD:", std, "MAX:", max, "MIN:", min, "RANGE:", max - min)
            print("SKEWBESS:", skewness)

        return skewness
