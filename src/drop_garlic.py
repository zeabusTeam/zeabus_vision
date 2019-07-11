#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
import math
from time import time
from sensor_msgs.msg import CompressedImage
from zeabus_utility.msg import VisionBox
from zeabus_utility.srv import VisionSrvDropGarlic
from constant import AnsiCode
from vision_lib import OutputTools, ImageTools,TransformTools
from geometry_msgs.msg import Point

image = ImageTools()
output = OutputTools('drop_garlic')
transform = TransformTools()

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
    if task == 'drop_garlic' :
        if request == 'search' :
            return find_drop_garlic()
        elif request == 'open' :
            return find_open()
        elif request == 'drop' :
            return find_drop()

def message(wimg,himg,state,cx1=0.0, cy1=0.0, cx2=0.0, cy2=0.0, cx3=0.0, cy3=0.0, cx4=0.0, cy4=0.0 , area=0.0):
    msg = VisionBox()
    msg.state = state
    msg.point_1 = (100*transform.convert(cx1,wimg) , -100.0*transform.convert(cy1,himg))
    msg.point_2 = (100*transform.convert(cx2,wimg) , -100.0*transform.convert(cy2,himg))
    msg.point_3 = (100*transform.convert(cx3,wimg) , -100.0*transform.convert(cy3,himg))
    msg.point_4 = (100*transform.convert(cx4,wimg) , -100.0*transform.convert(cy4,himg))
    msg.area = area*100
    print msg
    return msg

def get_cx(box,cnt,mode) :
    min_box = 1000
    max_box = 0
    for a in range(4) :
        sum_box = sum(box[a])
        if sum_box < min_box :
            min_box = sum_box
            cx4 = box[a][0]
            cy4 = box[a][1]
        if sum_box > max_box :
            max_box = sum_box
            cx2 = box[a][0]
            cy2 = box[a][1]
    if mode == 'find_mission' and (min_box == 0 or max_box == 1000) :
        return 0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
    min_y = 400
    max_y = 0
    for c in range(4) :
        if sum(box[c]) != cx2+cy2 and sum(box[c]) != cx4+cy4 :
            if box[c][1] < min_y :
                min_y = box[c][1]
                cx3 = box[c][0]
                cy3 = box[c][1]
            if box[c][1] > max_y :
                max_y = box[c][1]
                cx1 = box[c][0]
                cy1 = box[c][1]
    if mode == 'find_mission' and (max_y == 0 or min_y == 400) :
        return 0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0    
    if (mode == 'find_mission' and (cx1 <= 40 or cx2 >= 560 or cx3 >= 560 or cx4 <= 40 or cy1 >= 344 or cy2 >= 344 or cy3 <= 20 or cy4 <= 20 ) or mode == 'find_slice') and (cx1 <= 10 or cx2 >= 100 or cx3 >= 300 or cx4 <= 10 or cy1 >= 100 or cy2 >= 100 or cy3 <= 10 or cy4 <= 10 ):
    # if (cx1 <= 10 or cx2 >= 560 or cx3 >= 560 or cx4 <= 40 or cy1 >= 70 or cy2 >= 344 or cy3 <= 20 or cy4 <= 20 ):
        M = cv.moments(cnt)
        if M["m00"] != 0 :
            cx1 = int(M["m10"]/M["m00"])   
            cy1 = int(M["m01"]/M["m00"]) 
        else :
            cx1 = 0
            cy1 = 0
        return 2,cx1,cy1,0.0,0.0,0.0,0.0,0.0,0.0
    return 1,cx1,cy1,cx2,cy2,cx3,cy3,cx4,cy4

def find_drop_garlic () :
     
    image.to_gray()
    
    blur = cv.GaussianBlur(image.gray,(5,5),0)
    himg,wimg = blur.shape
    # print(r,c)
    blur = blur[20:-20,:]
    # equ = cv.equalizeHist(blur)
    # dist = cv.distanceTransform(otsu_th, cv.DIST_L2, 5)
    # _, img_dist = cv.threshold(dist, ret3, 1.0, cv.THRESH_BINARY)
    # print(dist.max())
    # print(dist.min())
    # print(img_dist.max())
    # print(img_dist.min())
    # img_dist = np.uint8(255*(img_dist-img_dist.min())/(img_dist.max()-img_dist.min()))
    # img_dist = np.uint8(img_dist)
    # test = image.bg_subtraction(mode='pos',bg_blur_size=15,fg_blur_size=11)
    blur = cv.GaussianBlur(image.gray,(5,5),0)
    ret1,th1 = cv.threshold(blur,blur.max()*0.7,255,cv.THRESH_BINARY)
    # print ret1
    # ret3,th3 = cv.threshold(blur,ret1,255,cv.THRESH_BINARY)
    # print ret3
    # ret,thresh1 = cv.threshold(blur,equ.max()*0.7,255,cv.THRESH_BINARY)
    # ret3,th3 = cv.threshold(blur,120,255,cv.THRESH_BINARY)
    # print ret1
    edges = cv.Canny(blur,ret1*9/10,ret1)
    output.publish(edges,'gray','/canny_be')
    # kernel = np.ones((7,7),np.uint8)
    # edges = cv.dilate(edges,kernel)
    output.publish(edges,'gray','/canny_af')
    # output.publish(ima,'gray','/1')
    # output.publish(th3,'gray','/th3')
    output.publish(blur,'gray','/2')
    output.publish(th1,'gray','/th1')
    # output.publish(otsu_th,'gray','/otsu')
    # output.publish(img_dist,'gray','/otsu_dist')
    # output.publish(img_dist,'gray','/otsu_dist')
    # output.publish(test,'gray','/test')
    # cnt = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    # edges = cv.bitwise_and(edges,th1)
    edges = cv.GaussianBlur(edges,(5,5),0)
    kernel = np.ones((1,11),np.uint8)
    edges = cv.dilate(edges,kernel)
    kernel = np.ones((11,1),np.uint8)
    edges = cv.dilate(edges,kernel)
    output.publish(edges,'gray','/test')
    # for i in edges :
    cnt = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    # print type(cnt)
    if len(cnt) > 0 :
        cnt = max(cnt,key=cv.contourArea)
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        approx = cv.approxPolyDP(cnt,0.01*cv.arcLength(cnt,True),True)
        # print len(approx)
        area = cv.contourArea(cnt)
        if area > 3000 and len(approx) <= 15:
            state,cx1,cy1,cx2,cy2,cx3,cy3,cx4,cy4 = get_cx(box,cnt,mode='find_mission')
            cv.drawContours(image.bgr,[box],0,(255,255,255),2)
            if state == 1 and ((cx2-cx1)/(cy3-cy2) >= 1.7 and  (cx2-cx1)/(cy3-cy2) <= 2.3):
                cv.drawContours(image.bgr,[box],0,(255,255,255),2)
                cv.circle(image.bgr, (cx4,cy4), 3, (255,0,0),-1)
                cv.circle(image.bgr, (cx2,cy2), 3, (0,255,0),-1)
                cv.circle(image.bgr, (cx3,cy3), 3, (0,0,255),-1)
                cv.circle(image.bgr, (cx1,cy1), 3, (0,0,0),-1)
            elif state == 2 :
                cv.circle(image.bgr, (cx1,cy1), 3, (255,255,255),-1)
            output.publish(image.bgr,'bgr','/cnt')
            return message(wimg,himg,state,cx1,cy1,cx2,cy2,cx3,cy3,cx4,cy4,area/(himg*wimg))
    # cv.drawContours(image.bgr, cnt, -1, (0,0,0), 3)
    return message(wimg,himg,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)

def find_drop() :

    image.to_gray()
    
    blur = cv.GaussianBlur(image.gray,(5,5),0)
    himg,wimg = blur.shape
    # print(r,c)
    blur = blur[20:-20,:]
    # equ = cv.equalizeHist(blur)
    # dist = cv.distanceTransform(otsu_th, cv.DIST_L2, 5)
    # _, img_dist = cv.threshold(dist, ret3, 1.0, cv.THRESH_BINARY)
    # print(dist.max())
    # print(dist.min())
    # print(img_dist.max())
    # print(img_dist.min())
    # img_dist = np.uint8(255*(img_dist-img_dist.min())/(img_dist.max()-img_dist.min()))
    # img_dist = np.uint8(img_dist)
    # test = image.bg_subtraction(mode='pos',bg_blur_size=15,fg_blur_size=11)
    blur = cv.GaussianBlur(image.gray,(5,5),0)
    ret1,th1 = cv.threshold(blur,blur.max()*0.7,255,cv.THRESH_BINARY)
    # print ret1
    # ret3,th3 = cv.threshold(blur,ret1,255,cv.THRESH_BINARY)
    # print ret3
    # ret,thresh1 = cv.threshold(blur,equ.max()*0.7,255,cv.THRESH_BINARY)
    # ret3,th3 = cv.threshold(blur,120,255,cv.THRESH_BINARY)
    # print ret1
    edges = cv.Canny(blur,ret1*9/10,ret1)
    output.publish(edges,'gray','/canny_be')
    # kernel = np.ones((7,7),np.uint8)
    # edges = cv.dilate(edges,kernel)
    output.publish(edges,'gray','/canny_af')
    # output.publish(ima,'gray','/1')
    # output.publish(th3,'gray','/th3')
    output.publish(blur,'gray','/2')
    output.publish(th1,'gray','/th1')
    # output.publish(otsu_th,'gray','/otsu')
    # output.publish(img_dist,'gray','/otsu_dist')
    # output.publish(img_dist,'gray','/otsu_dist')
    # output.publish(test,'gray','/test')
    # cnt = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    # edges = cv.bitwise_and(edges,th1)
    edges = cv.GaussianBlur(edges,(5,5),0)
    kernel = np.ones((1,11),np.uint8)
    edges = cv.dilate(edges,kernel)
    kernel = np.ones((11,1),np.uint8)
    edges = cv.dilate(edges,kernel)
    output.publish(edges,'gray','/test')
    # for i in edges :
    cnt = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    # print type(cnt)
    if len(cnt) > 0 :
        cnt = max(cnt,key=cv.contourArea)
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        state,cx1,cy1,cx2,cy2,cx3,cy3,cx4,cy4 = get_cx(box,cnt,mode='find_mission')
        area = cv.contourArea(cnt)
        approx = cv.approxPolyDP(cnt,0.01*cv.arcLength(cnt,True),True)
        if area > 3000 and len(approx) <= 15:
            gray = image.gray[max(int(cy4),int(cy3)):min(int(cy1),int(cy2)),max(int(cx1),int(cx4)):min(int(cx2),int(cx3))]
            output.publish(gray,'gray','/test')
            ret1,th1 = cv.threshold(gray,gray.max()*0.7,255,cv.THRESH_BINARY)
            th1 = cv.bitwise_not(th1)
            output.publish(th1,'gray','/th1')
            cnt = cv.findContours(gray, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
            # print type(cnt)
            if len(cnt) > 0 :
                cnt = max(cnt,key=cv.contourArea)
                area_slice = cv.contourArea(cnt)
                rect = cv.minAreaRect(cnt)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                state,cx1_d,cy1_d,cx2_d,cy2_d,cx3_d,cy3_d,cx4_d,cy4_d = get_cx(box,cnt,mode = 'find_slice')
                # print cy1
                # print cy1_d
                # print ('-----')
                cx1 += cx1_d
                cy1 -= cy1_d
                cx2 -= cx2_d
                cy2 -= cy2_d
                cx3 -= cx3_d
                cy3 += cy3_d
                cx4 += cx4_d
                cy4 -= cy4_d
                box[0][0] = cx1
                box[0][1] = cy1
                box[1][0] = cx2
                box[1][1] = cy2
                box[2][0] = cx3
                box[2][1] = cy3
                box[3][0] = cx4
                box[3][1] = cy4
                # cv.drawContours(image.bgr,[[[cx1,cy1],[cx2,cy2],[cx3,cy3],[cx4,cy4]]],0,(255,255,255),2)
                # cv.drawContours(image.bgr,[box],0,(255,255,255),2)
                if state == 1 and ((cx2-cx1)/(cy3-cy2) >= 0.2 and  (cx2-cx1)/(cy3-cy2) <= 0.7):
                    cv.drawContours(image.bgr,[box],0,(255,255,255),2)
                    cv.circle(image.bgr, (cx4,cy4), 3, (255,0,0),-1)
                    cv.circle(image.bgr, (cx2,cy2), 3, (0,255,0),-1)
                    cv.circle(image.bgr, (cx3,cy3), 3, (0,0,255),-1)
                    cv.circle(image.bgr, (cx1,cy1), 3, (0,0,0),-1)
                elif state == 2 :
                    cv.circle(image.bgr, (cx1,cy1), 5, (255,255,255),-1)
                output.publish(image.bgr,'bgr','/cnt')
                return message(wimg,himg,state,cx1,cy1,cx2,cy2,cx3,cy3,cx4,cy4,area_slice/(himg*wimg))
    # cv.drawContours(image.bgr, cnt, -1, (0,0,0), 3)
    return message(wimg,himg,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)


if __name__ == '__main__':
    rospy.init_node('vision_drop_garlic', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber('/vision/bottom/image_raw/compressed', CompressedImage, image.callback)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    rospy.Service('/vision/drop_garlic',
                  VisionSrvDropGarlic(), mission_callback)
    output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
