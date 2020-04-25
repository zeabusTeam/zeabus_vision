import cv2 as cv
import rospy
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from zeabus_utility.msg import VisionBox
from constant import AnsiCode

import os


class OutputTools:
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.topic = self.process_topic(topic)
        self.end_loop = True

    def process_topic(self, topic):
        '''
            Example topic: '/vision/gate'

            if input is '':
                return /vision/empty_topic/
            else if input is '/vision/gate':
                return input
            else if input is 'vision/gate':
                return '/' + input              # '/' + 'vision/gate' = '/vision/gate'
            else if input is 'gate':
                return '/vision/' + input       # '/vision/' + 'gate' = '/vision/gate'
            else if input is '/gate':
                return '/vision' + input        # '/vision' + '/gate' = '/vision/gate'
        '''
        if topic == '':
            return '/vision/empty_topic/'
        elif topic[:7] == '/vision':
            return str(topic)
        elif topic[:6] == 'vision':
            return '/' + str(topic)
        elif topic[0] != '/':
            return '/vision/' + str(topic)
        else:
            return '/vision' + str(topic)

    def log(self, msg, color=AnsiCode.DEFAULT):
        """
            print ('<----------') + str(msg) + ('---------->')
            # len of <---msg---> = 80
        """
        if self.end_loop:
            os.system('clear')
        white_character = len(color) + 80
        temp = '<{:-^' + str(white_character) + '}>'
        print (temp.format(' ' + color +
                           str(msg) + AnsiCode.DEFAULT + ' '))
        self.end_loop = False

    def img_is_none(self):
        if self.end_loop:
            os.system('clear')
        print(AnsiCode.RED + 'img is none.'+'\n'
              'Please check topic name or check camera is running' +
              AnsiCode.DEFAULT)
        self.end_loop = False

    def create_publish(self, datatype, subtopic):
        '''
            Expect subtopic example: '/image_result'
            if input is 'image_result':
                new subtopic = '/' + input      # '/' + 'image_result' = 'image_result'
        '''
        if subtopic[0] != '/':
            subtopic = '/' + subtopic
        return rospy.Publisher(self.topic + str(subtopic), datatype, queue_size=10)

    def publish_image(self, img, color, publisher=None, subtopic=None):
        """
            publish picture
        """
        if publisher == None:
            publisher = self.create_publish(datatype=Image, subtopic=subtopic)
        if img is None:
            img = np.zeros((200, 200))
            color = "gray"
        if color == 'gray':
            msg = self.bridge.cv2_to_imgmsg(img, "mono8")
        elif color == 'bgr':
            msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        publisher.publish(msg)


class TransformTools:
    def __init__(self):
        pass

    def convert(self, inp, full):
        inp = float(inp)
        full = float(full)
        res = (inp - (full / 2.0)) / (full / 2.0)
        return res

    def to_point(self, x, y, shape, z=0.0):
        himg, wimg = shape[:2]
        point = Point()
        point.x = float(self.convert(x, wimg))
        point.y = float(-1.0*self.convert(y, himg))
        point.z = float(z)
        return point

    def convert_to_point(self, pt, shape):
        himg, wimg = shape[:2]
        x = self.convert(pt[0], wimg)
        y = -1.0*self.convert(pt[1], himg)
        return [x, y]

    def to_box(self, pt1, pt2, pt3, pt4, area, shape, state=0):
        himg, wimg = shape[:2]
        box = VisionBox()
        box.point_1 = self.convert_to_point(pt1, shape)
        box.point_2 = self.convert_to_point(pt2, shape)
        box.point_3 = self.convert_to_point(pt3, shape)
        box.point_4 = self.convert_to_point(pt3, shape)
        box.area = self.convert(area, himg*wimg)
        box.state = state
        return box


class ImageTools:
    class topic:
        FRONT_RECT = '/vision/front/image_rect_color/compressed'
        FRONT_RAW = '/vision/front/image_raw/compressed'
        FRONT = FRONT_RECT

        BOTTOM_RAW = '/vision/bottom/image_raw/compressed'
        BOTTOM = BOTTOM_RAW

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


    def renew_display(self):
        self.display = self.bgr.copy()

    def normalize(self, gray):
        return np.uint8(255 * (gray - gray.min()) / (gray.max() - gray.min()))

    def bg_subtraction(self, mode='neg', bg_blur_size=61, fg_blur_size=5):
        """
            new bg_subtraction
            create by: skconan
        """
        self.to_gray()
        bg = cv.medianBlur(self.gray, bg_blur_size)
        fg = cv.medianBlur(self.gray, fg_blur_size)
        sub_sign = np.int16(fg) - np.int16(bg)
        if mode == 'neg':
            sub_neg = np.clip(sub_sign.copy(), sub_sign.copy().min(), 0)
            sub_neg = self.normalize(sub_neg)
            _, obj = cv.threshold(
                sub_neg, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)
            return obj
        elif mode == 'pos':
            sub_pos = np.clip(sub_sign.copy(), 0, sub_sign.copy().max())
            sub_pos = self.normalize(sub_pos)
            _, obj = cv.threshold(
                sub_pos, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
            return obj
        return self.gray

    def get_kernel(self, shape='rect', ksize=(5, 5)):
        if shape == 'rect':
            return cv.getStructuringElement(cv.MORPH_RECT, ksize)
        if shape == 'ellipse':
            return cv.getStructuringElement(cv.MORPH_ELLIPSE, ksize)
        if shape == 'plus':
            return cv.getStructuringElement(cv.MORPH_CROSS, ksize)
        return None

    def kmean(self, img, k, max_iter):
        criteria = (cv.TERM_CRITERIA_EPS +
                    cv.TERM_CRITERIA_MAX_ITER, max_iter, 1.0)
        Z = img.reshape((-1, 1))

        # convert to np.float32
        Z = np.float32(Z)

        K = k
        ret, label, center = cv.kmeans(
            Z, K, None, criteria, max_iter, cv.KMEANS_RANDOM_CENTERS)

        # Now convert back into uint8, and make original image
        center = np.uint8(center)
        res = center[label.flatten()]
        result = res.reshape((img.shape))

        return result

    def bg_subtraction_kmean(self, img, bg_k=1, fg_k=3, mode='neg', max_iter=5):
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        gray, _, _ = cv.split(hsv)

        # start_time = rospy.Time.now()
        bg = self.kmean(gray, k=bg_k, max_iter=max_iter)
        fg = self.kmean(gray, k=fg_k, max_iter=max_iter)

        sub_sign = np.int16(fg) - np.int16(bg)

        if mode == 'neg':
            sub_neg = np.clip(sub_sign.copy(), sub_sign.copy().min(), 0)
            sub_neg = self.normalize(sub_neg)
            _, result = cv.threshold(
                sub_neg, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU
            )
        else:
            sub_pos = np.clip(sub_sign.copy(), 0, sub_sign.copy().max())
            sub_pos = self.normalize(sub_pos)
            _, result = cv.threshold(
                sub_pos, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU
            )

        # time_duration = rospy.Time.now()-start_time
        # print(time_duration.to_sec())

        return result


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
        Max = count.max()
        count = list(count)
        return count.index(Max)

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


class Detector:
    ''' make some constant don't repeat run'''

    def __init__(self, picture_name, min_match=20):
        import os
        self.PICTURE_NAME = 'pictures/' + picture_name
        self.MIN_MATCH_COUNT = min_match
        self.FLANN_INDEX_KDITREE = 0
        self.sift = cv.xfeatures2d.SIFT_create()
        self.file_dir = os.path.dirname(os.path.abspath(__file__))
        self.picture_dir = os.path.join(self.file_dir, self.PICTURE_NAME)
        self.picture = cv.imread(self.picture_dir)
        self.flannParam = dict(algorithm=self.FLANN_INDEX_KDITREE, tree=5)
        self.flann = cv.FlannBasedMatcher(self.flannParam, {})
        self.train_keypoint, self.train_desc = self.compute(self.picture)

    def compute(self, img):
        result = self.sift.detectAndCompute(img, None)
        return result

    def get_train_border(self):
        h, w = self.picture.shape[:2]
        trainBorder = np.float32([[[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]])
        return trainBorder
