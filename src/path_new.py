#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
import math
from time import time
from sensor_msgs.msg import CompressedImage
from zeabus_utility.msg import VisionPath
from zeabus_utility.srv import VisionSrvPath
from constant import AnsiCode
from vision_lib import OutputTools, ImageTools , RosCmd

image = ImageTools()
output = OutputTools()
pub = RosCmd()

class Log:
    def __init__(self):
        self.time = time()

    def reset_all(self):
        pass


def mission_callback(msg):
    """
        When call service it will run this 
        Returns:
            a group of process value from this program
    """
    if image.bgr is None:
        output.img_is_none()
        return None
    task = str(msg.task.data)
    request = str(msg.request.data)
    if task == 'path' and request != None :
        return find_path() 


def message(cx1=0.0, cy1=0.0, cx2=0.0, cy2=0.0, cx3=0.0, cy3=0.0, n=0 , area1 = 0.0 , area2 = 0.0):
    msg = VisionPath()
    msg.point1 = tuple(cx1,cy1)
    msg.point2 = tuple(cx2,cy2)
    msg.point3 = tuple(cx3,cy3)
    msg.n_point = int(n)
    msg.area1 = float(area1)
    msg.area2 = float(area2)
    return msg


def get_mask():
    # test = image.bg_subtraction(mode='pos',bg_blur_size=211,fg_blur_size=11)
    # pub.publish_result(test,'gray','/bg_sub')
    image.to_hsv()
    upper = np.array([90, 255, 255], dtype=np.uint8)
    lower = np.array([28, 0, 0], dtype=np.uint8)
    mask = cv.inRange(image.hsv, lower, upper)

    # image.to_gray()
    pub.publish_result(mask,'gray','/mask')
    test = image.bg_subtraction(mode='pos',bg_blur_size=211,fg_blur_size=11)
    pub.publish_result(test,'gray','/bg_sub')
    test[test >= 250] = 255
    test[test < 250] = 0
    # image.gray[image.gray >= 100] = 255
    # image.gray[image.gray < 100] = 0
    # mask = test
    # image.gray = cv.bitwise_not(image.gray)
    mask = cv.bitwise_not(mask)
    mask = cv.bitwise_and(test,mask)

    pub.publish_result(image.gray,'gray','/path')
    mask = cv.bitwise_and(mask,test)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv.GaussianBlur(mask, (5, 5), 0)
    pub.publish_result(mask,'gray','/test')
    mask = cv.erode(mask, kernel)
    mask = cv.dilate(mask, kernel)
    pub.publish_result(mask,'gray','/path/ed')
    return mask


def get_obj(mask):
    himg, wimg = img.shape[:2]
    cnt = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    for i in cnt :
        if cv.contourArea(i) < 200 :
            pass
        else :
            x,y,w,h = cv2.boundingRect(i)
            t_top = y  
            b_top = y+(h/10)
            t_mid = ((y+h)/2)-(h/10)
            b_mid = ((y+h)/2)+(h/10)
            t_bot = y+h-(h/10)
            b_bot = y+h
            crop_t = mask[t_top:b_top,x:x+w]
            crop_m = mask[t_mid:b_mid,x:x+w]
            crop_b = mask[t_bot:b_bot,x:x+w]
            c_top = cv.findContours(crop_t, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
            c_middle = cv.findContours(crop_m, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
            c_bottom = cv.findContours(crop_b, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
            M_top = cv.moment(c_top)
            M_bottom = cv.moment(c_middle)
            M_bot = cv.moments(c_bottom)
            cx_top = int(M_top["m10"]/M_top["m00"])
            cy_top = int(M_top["m01"]/M_top["m00"])
            cx_mid = int(M_mid["m10"]/M_mid["m00"])
            cy_mid = int(M_mid["m01"]/M_mid["m00"])
            cx_bot = int(M_bot["m10"]/M_bot["m00"])
            cy_bot = int(M_bot["m01"]/M_bot["m00"])
            if (h/w >= 1 and h/w <= 1.9) :
                y_crop = y+(h/w)*100
                crop_top = mask[y:y+h/2,x:x+w]
                crop_bot = mask[y+h/2:y+h,w:x+w]
                cnt_top = cv.findContours(crop_top, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
                cnt_bot = cv.findContours(crop_bot, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
                area1 = cv.contourArea(cnt_bot)/(himg*wimg)
                area2 = cv.contourArea(cnt_top)/(himg*wimg)
                if math.sqrt((((cx_top-cx_bot)**2) + ((cy_top-cy_bot)**2))) >= 45 and math.sqrt((((cx_top-cx_bot)**2) + ((cy_top-cy_bot)**2))) <= 49 :
                    return cx1=cx_bot, cy1=cy_bot, cx2=cx_mid, cy2=cy_mid, cx3=cx_top, cy3=cy_bot, n=3 , area1 = area1 , area2 = area2
            else :
                if y == 0 and h <= himg/3:
                    area1 = cv.contourArea()
                    return cx1=cx_bot, cy1=cy_bot, cx2=cx_top, cy2=cy_top, cx3=0.0, cy3=0.0, n=2 , area1 = area1 , area2 = 0.0
                    
                    
                # rect = cv.minAreaRect(cnt)
                # box = cv.boxPoints(rect)
                # box = np.int0(box)
                
    return -1,-1,-1,-1,-1,-1 

def find_path():
    mask = get_mask()
    obj = get_obj(mask)
    return message()


if __name__ == '__main__':
    rospy.init_node('vision_path', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber('/bottom/left/image_raw/compressed', CompressedImage, image.callback)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    rospy.Service('/vision/path',
                  VisionSrvPath(), mission_callback)
    output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
