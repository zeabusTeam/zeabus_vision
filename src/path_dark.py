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
from vision_lib import OutputTools, ImageTools,TransformTools
from geometry_msgs.msg import Point

image = ImageTools()
output = OutputTools('path')
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
    if task == 'path' and request != None :
        return find_path() 


def message(wimg,himg,cx1=0.0, cy1=0.0, cx2=0.0, cy2=0.0, cx3=0.0, cy3=0.0, n_point=0 , area1 = 0.0 , area2 = 0.0):
    msg = VisionPath()
    msg.point_1 = (100*transform.convert(cx1,wimg) , -100.0*transform.convert(cy1,himg))
    msg.point_2 = (100*transform.convert(cx2,wimg) , -100.0*transform.convert(cy2,himg))
    msg.point_3 = (100*transform.convert(cx3,wimg) , -100.0*transform.convert(cy3,himg))
    msg.n_point = int(n_point)
    msg.area = (area1*100,area2*100)
    print msg
    return msg


def get_mask():
    # test = image.bg_subtraction(mode='pos',bg_blur_size=211,fg_blur_size=11)
    # pub.publish_result(test,'gray','/bg_sub')
    image.to_hsv()
    upper = np.array([238, 255, 255], dtype=np.uint8)
    lower = np.array([50, 0, 0], dtype=np.uint8)
    mask = cv.inRange(image.hsv, lower, upper)
    mask = cv.medianBlur(mask,5)
    mask = cv.bitwise_not(mask)
    output.publish(mask,'gray','/mask')

    # image.to_gray()
    # pub.publish_result(mask,'gray','/mask')
    # test = image.bg_subtraction(mode='pos',bg_blur_size=211,fg_blur_size=11)
    # pub.publish_result(test,'gray','/bg_sub')
    # test[test >= 250] = 255
    # test[test < 250] = 0
    # image.gray[image.gray >= 100] = 255
    # image.gray[image.gray < 100] = 0
    # mask = test
    # image.gray = cv.bitwise_not(image.gray)
    # mask = cv.bitwise_not(mask)
    # mask = cv.bitwise_and(test,mask)

    # pub.publish_result(image.gray,'gray','/path')
    # mask = cv.bitwise_and(mask,test)
    # kernel = np.ones((5, 5), dtype=np.uint8)
    # mask = cv.GaussianBlur(mask, (5, 5), 0)
    # pub.publish_result(mask,'gray','/test')
    # mask = cv.erode(mask, kernel)
    # mask = cv.dilate(mask, kernel)
    # pub.publish_result(mask,'gray','/path/ed')
    return mask


def get_obj(mask):
    cx = []
    cy = []
    cx.append(0)
    cy.append(0)
    angle = []
    angle.append(0)
    turn_point = 0
    old_ang = 0
    himg, wimg = mask.shape[:2]
    cnt = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    if len(cnt) == 0:
		return wimg,himg,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
    cnt = max(cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1],key=cv.contourArea)
    if cv.contourArea(cnt) < 1500:
    	return wimg,himg,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
    x,y,w,h = cv.boundingRect(cnt)
    if h < w*3/4 :
    	return wimg,himg,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
    # cv.rectangle(mask,(x,y),(x+w,y+h),(255,255,255),2)
    y_bot_crop = y
    for i in range (1,11) :
        y_top_crop = y_bot_crop
        y_bot_crop = y+(h*i/10)
        # print i
        # print h
        # print y_top_crop
        # print y_bot_crop
        crop = mask[y_top_crop:y_bot_crop,x:x+w]
        cnt_crop = max(cv.findContours(crop, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1],key=cv.contourArea)
        # cv.rectangle(mask,(x,y_top_crop),(x+w,y_bot_crop),(255,255,255),2)
        M = cv.moments(cnt_crop)
        if M["m00"] != 0 :
            cx_crop = int(M["m10"]/M["m00"]) + x  
            cy_crop = int(M["m01"]/M["m00"]) + y_top_crop
        else :
            cx_crop = 0
            cy_crop = 0
        # cv.circle(mask,(int(cx_crop),int(cy_crop)),3,(0, 255, 255), -1)
        # pub.publish_result(mask,'gray','/p')
        cx.append(cx_crop)
        cy.append(cy_crop)
        # print ("cy[i] = " + str(cy[i]) )
        # print ("cy[i-1] = " + str(cy[i-1]) )
        # print ("cx[i] = " + str(cx[i]) )
        # print ("cx[i-1] = " + str(cx[i-1]) )
        ang_crop = math.atan2(cy[i]-cy[i-1],cx[i]-cx[i-1])
        ang_crop = math.degrees(ang_crop)
        # ang_crop = math.atan2(cy[i] - cy[i-1],cx[i] - cx[i-1])*180/math.pi
        angle.append(ang_crop)
        diff = ang_crop - old_ang
        # print ("now = " + str(ang_crop) + " pass = " + str(old_ang) + " diff = " + str(diff))
        if (i > 2) and (abs(ang_crop - old_ang) > 20) :
            turn_point = i-1
        old_ang = ang_crop
    if turn_point != 0 :
        first_ang = math.atan2(cy[1]-cy[turn_point],cx[1]-cx[turn_point])
        first_ang = math.degrees(first_ang)
        last_ang = math.atan2(cy[turn_point]-cy[10],cx[turn_point]-cx[10])
        last_ang = math.degrees(last_ang)
        # print ("f - l = " + str((abs(first_ang - last_ang))))
        if (abs(first_ang - last_ang) < 34) :
            turn_point = 0
    if turn_point == 0 :
        n_point = 2
        area1 = cv.contourArea(cnt)/(himg*wimg)
        area2 = 0.0
        cx1 = cx[10]
        cy1 = cy[10]
        cx2 = cx[1]
        cy2 = cy[1]
        cx3 = 0.0
        cy3 = 0.0
    else :
        n_point = 3
        # crop_top = mask[y:y+(turn_point*h/10),x:x+w]
        # cnt_top = max(cv.findContours(crop_top, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1],key=cv.contourArea)
        # area1 = cv.contourArea(cnt_top)/(himg*wimg)
        # crop_bot = mask[y+(turn_point*h/10):y+h,x:x+w]
        # cnt_bot = max(cv.findContours(crop_bot, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1],key=cv.contourArea)
        # area2 = cv.contourArea(cnt_bot)/(himg*wimg)
        # cv.rectangle(mask,(x,y),(x+w,y+(turn_point*h/10)),(255,255,255),2)
        # cv.rectangle(mask,(x,y+(turn_point*h/10)),(x+w,y+h),(255,255,255),2)
        cx1 = cx[10]
        cy1 = cy[10]
        cx3 = cx[1]
        cy3 = cy[1]
        cx2 = cx[turn_point]
        cy2 = cy[turn_point]
        crop_top = mask[y:cy2,x:x+w]
        cnt_top = max(cv.findContours(crop_top, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1],key=cv.contourArea)
        area2 = cv.contourArea(cnt_top)/(himg*wimg)
        crop_bot = mask[cy2:y+h,x:x+w]
        cnt_bot = max(cv.findContours(crop_bot, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1],key=cv.contourArea)
        area1 = cv.contourArea(cnt_bot)/(himg*wimg)
        cv.rectangle(mask,(x,y),(x+w,cy2),(255,255,255),2)
        cv.rectangle(mask,(x,cy2),(x+w,y+h),(255,255,255),2)
    
    if cx1 > wimg : 
        cx1 = wimg 
    elif cx1 < 0 :
        cx1 = 0
    if cx2 > wimg : 
        cx2 = wimg 
    elif cx2 < 0 :
        cx2 = 0
    if cx3 > wimg : 
        cx3 = wimg
    elif cx3 < 0 :
        cx3 = 0 
    if cy1 > wimg : 
        cy1 = wimg
    elif cy1 < 0 :
        cy1 = 0 
    if cy2 > wimg : 
        cy2 = wimg
    elif cy2 < 0 :
        cy2 = 0 
    if cy3 > wimg : 
        cy3 = wimg
    elif cy3 < 0 :
        cy3 = 0  
    # print turn_point 
    cv.circle(mask,(int(cx1),int(cy1)),3,(0, 255, 255), -1)
    cv.circle(mask,(int(cx2),int(cy2)),3,(0, 255, 255), -1)
    cv.circle(mask,(int(cx3),int(cy3)),3,(0, 255, 255), -1)
    output.publish(mask,'gray','/p')
    return wimg,himg,cx1,cy1,cx2,cy2,cx3,cy3,area1,area2,n_point
            # t_top = y  
            # b_top = y+(h/10)
            # t_mid = ((y+h)/2)-(h/10)
            # b_mid = ((y+h)/2)+(h/10)
            # t_bot = y+h-(h/10)
            # b_bot = y+h
            # crop_t = mask[t_top:b_top,x:x+w]
            # crop_m = mask[t_mid:b_mid,x:x+w]
            # crop_b = mask[t_bot:b_bot,x:x+w]
            # c_top = cv.findContours(crop_t, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[2]
            # c_middle = cv.findContours(crop_m, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[2]
            # c_bottom = cv.findContours(crop_b, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[2]
            # M_top = cv.moment(c_top)
            # M_bottom = cv.moment(c_middle)
            # M_bot = cv.moments(c_bottom)
            # cx_top = int(M_top["m10"]/M_top["m00"])
            # cy_top = int(M_top["m01"]/M_top["m00"])
            # cx_mid = int(M_mid["m10"]/M_mid["m00"])
            # cy_mid = int(M_mid["m01"]/M_mid["m00"])
            # cx_bot = int(M_bot["m10"]/M_bot["m00"])
            # cy_bot = int(M_bot["m01"]/M_bot["m00"])
            # if (max(h,w)/min(h,w) >= 1.6 and max(h,w)/min(h,w) <= 2.0) :
            #     #FULL PATH
            #     if (h > w) :
            #         if math.atan2(cy_top - cy_bot, abs(cx_top - cx_bot)) * 180 / math.PI;
            #     crop_top = mask[y:y+h/2,x:x+w]
            #     crop_bot = mask[y+h/2:y+h,w:x+w]
            #     cnt_top = cv.findContours(crop_top, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[2]
            #     cnt_bot = cv.findContours(crop_bot, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[2]
            #     area1 = cv.contourArea(cnt_bot)/(himg*wimg)
            #     area2 = cv.contourArea(cnt_top)/(himg*wimg)
            #     if math.sqrt((((cx_top-cx_bot)**2) + ((cy_top-cy_bot)**2))) >= 45 and math.sqrt((((cx_top-cx_bot)**2) + ((cy_top-cy_bot)**2))) <= 49 :
            #         return cx1=cx_bot, cy1=cy_bot, cx2=cx_mid, cy2=cy_mid, cx3=cx_top, cy3=cy_bot, n=3 , area1 = area1 , area2 = area2
            # else :
            #     if y == 0 and h <= himg/3:
            #         area1 = cv.contourArea()
            #         return cx1=cx_bot, cy1=cy_bot, cx2=cx_top, cy2=cy_top, cx3=0.0, cy3=0.0, n=2 , area1 = area1 , area2 = 0.0
                    
                    
                # rect = cv.minAreaRect(cnt)
                # box = cv.boxPoints(rect)
                # box = np.int0(box)
                

def find_path():
    mask = get_mask()
    wimg , himg ,cx1 , cy1 , cx2 ,cy2 , cx3 ,cy3 , area1 , area2 , n_point = get_obj(mask)
    return message(wimg=wimg,himg=himg,cx1=cx1,cy1=cy1,cx2=cx2,cy2=cy2,cx3=cx3,cy3=cy3,area1=area1,area2=area2,n_point=n_point)

if __name__ == '__main__':
    rospy.init_node('vision_path', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber('/vision/bottom/image_raw/compressed', CompressedImage, image.callback)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    rospy.Service('/vision/path',
                  VisionSrvPath(), mission_callback)
    output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
