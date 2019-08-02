#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
import math
from time import time
from sensor_msgs.msg import CompressedImage
from zeabus_utility.msg import VisionBox
from zeabus_utility.srv import VisionSrvDropGarlic, VisionSrvDropGarlicResponse
from constant import AnsiCode
from vision_lib import OutputTools, ImageTools, TransformTools
from geometry_msgs.msg import Point
from vision_lib import OutputTools, ImageTools, TransformTools, Detector


image = ImageTools(sub_sampling=0.3)
output = OutputTools(topic='/vision/drop/')
transform = TransformTools()
detector = Detector(picture_name='bat-full-usa.png', min_match=27)
seq = 1


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
    print request
    if task == 'drop_garlic':
        if request == 'search':
            return find_drop_garlic(func='search')
        elif request == 'open':
            return find_open()
        elif request == 'drop':
            return find_drop()


def to_box(state=0, box=0, color=(0, 255, 0), area=0.0, center=True):
    shape = image.display.shape[:2]
    print('shape', shape)
    sort = sorted(box, key=lambda x: x[0])
    bottom = sort[:2]
    bottom = sorted(bottom, key=lambda x: x[1])
    pt4 = bottom[0]
    pt1 = bottom[1]
    top = sort[2:]
    top = sorted(top, key=lambda x: x[1])
    pt3 = top[0]
    pt2 = top[1]
    msg = VisionBox()
    msg.state = state
    print(shape[0]*shape[1], 'shape')
    if area == -1:
        msg.area = q_area(box)/(shape[0]*shape[1])
    elif area > 0:
        msg.area = area/(shape[0]*shape[1])

    if center:
        cx = cy = 0
        for pt in box:
            cx += pt[0]
            cy += pt[1]
        cx /= 4
        cy /= 4
        cv.putText(image.display, 'c', (cx, cy),
                   cv.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv.LINE_AA)
        cv.circle(image.display, (cx, cy), 5, color, -1)
        msg.point_1 = transform.convert_to_point((cx, cy), shape)
        print('msg', msg)
    else:
        for i in range(1, 5):
            print('pt'+str(i), tuple(eval('pt'+str(i))))
            cv.putText(image.display, str(i), tuple(eval('pt'+str(i))),
                       cv.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv.LINE_AA)
        msg.point_1 = transform.convert_to_point(pt1, shape)
        msg.point_2 = transform.convert_to_point(pt2, shape)
        msg.point_3 = transform.convert_to_point(pt3, shape)
        msg.point_4 = transform.convert_to_point(pt4, shape)

    return msg


def a_message(state=0, box=0, area=0.0, center=True):
    response = VisionSrvDropGarlicResponse()
    if state < 0:
        return response
    elif state >= 1:
        response.data = to_box(state=state, box=box,
                               area=area, center=center)
    output.log('Publishing display', AnsiCode.GREEN)
    output.publish(image.display, 'bgr', 'display')
    print(response)
    return response


def q_area(box, n=4):
    area = 0.0
    j = n-1
    for i in range(0, n):
        area += (box[j][0] + box[i][0]) * (box[j][1] - box[i][1])
        j = i
    print(area)
    return abs(area / 2.0)


def message(wimg, himg, state, cx1=0.0, cy1=0.0, cx2=0.0, cy2=0.0, cx3=0.0, cy3=0.0, cx4=0.0, cy4=0.0, area=0.0):
    msg = VisionBox()
    msg.state = state
    msg.point_1 = (100*transform.convert(cx1, wimg), -
                   100.0*transform.convert(cy1, himg))
    msg.point_2 = (100*transform.convert(cx2, wimg), -
                   100.0*transform.convert(cy2, himg))
    msg.point_3 = (100*transform.convert(cx3, wimg), -
                   100.0*transform.convert(cy3, himg))
    msg.point_4 = (100*transform.convert(cx4, wimg), -
                   100.0*transform.convert(cy4, himg))
    msg.area = area*100
    print msg
    return msg


def get_mask():
    image.renew_display()
    # test = image.bg_subtraction(mode='pos',bg_blur_size=211,fg_blur_size=11)
    # pub.publish_result(test,'gray','/bg_sub')
    max_iter = 5
    # publish_result(obj_pos, "gray", "/path_obj_pos")
    mask = image.bg_subtraction_kmean(
        image.display, bg_k=1, fg_k=3, mode='pos', max_iter=max_iter)
    # return mask
    # upper = np.array([238, 255, 255], dtype=np.uint8)
    # lower = np.array([50, 0, 0], dtype=np.uint8)
    # mask = cv.inRange(image.hsv, lower, upper)
    # mask = cv.medianBlur(mask,5)
    # mask = cv.bitwise_not(mask)
    # test = image.bg_subtraction(mode='pos',bg_blur_size=211,fg_blur_size=11)
    # pub.publish_result(test,'gray','/bg_sub')
    # image.to_hsv()
    #upper = np.array([238, 255, 255], dtype=np.uint8)
    #lower = np.array([50, 0, 0], dtype=np.uint8)
    #mask = cv.inRange(image.hsv, lower, upper)
    #mask = cv.medianBlur(mask,5)
    #mask = cv.bitwise_not(mask)
    output.publish(mask, 'gray', '/mask')

    image.to_gray()
    output.publish(image.gray, 'gray', '/gray')
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


def get_cx(box, cnt, mode):
    min_box = 1000
    max_box = 0
    for a in range(4):
        sum_box = sum(box[a])
        if sum_box < min_box:
            min_box = sum_box
            cx4 = box[a][0]
            cy4 = box[a][1]
        if sum_box > max_box:
            max_box = sum_box
            cx2 = box[a][0]
            cy2 = box[a][1]
    if mode == 'find_mission' and (min_box == 0 or max_box == 1000):
        return 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    min_x = 1000
    max_x = 0
    for c in range(4):
        if sum(box[c]) != cx2+cy2 and sum(box[c]) != cx4+cy4:
            if box[c][0] < min_x:
                min_x = box[c][0]
                cx1 = box[c][0]
                cy1 = box[c][1]
            if box[c][0] > max_x:
                max_x = box[c][0]
                cx3 = box[c][0]
                cy3 = box[c][1]
    if (max_x == 0 or min_x == 1000):
        return 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    if (mode == 'find_mission' and (cx1 <= 40 or cx2 >= 560 or cx3 >= 560 or cx4 <= 40 or cy1 >= 344 or cy2 >= 344 or cy3 <= 20 or cy4 <= 20)) or (mode == 'find_slice' and (cx1 <= 10 or cx2 >= 100 or cx3 >= 300 or cx4 <= 10 or cy1 >= 100 or cy2 >= 100 or cy3 <= 10 or cy4 <= 10)):
        M = cv.moments(cnt)
        if M["m00"] != 0:
            cx1 = int(M["m10"]/M["m00"])
            cy1 = int(M["m01"]/M["m00"])
        else:
            cx1 = 0
            cy1 = 0
        return 2, cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4
    return 1, cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4


# def find_drop_garlic(func):
#     image.renew_display()
#     image.to_gray()

#     blur = cv.GaussianBlur(image.gray, (5, 5), 0)
#     himg, wimg = blur.shape
#     # print(r,c)
#     blur = blur[20:-20, :]
#     # equ = cv.equalizeHist(blur)
#     # dist = cv.distanceTransform(otsu_th, cv.DIST_L2, 5)
#     # _, img_dist = cv.threshold(dist, ret3, 1.0, cv.THRESH_BINARY)
#     # print(dist.max())
#     # print(dist.min())
#     # print(img_dist.max())
#     # print(img_dist.min())
#     # img_dist = np.uint8(255*(img_dist-img_dist.min())/(img_dist.max()-img_dist.min()))
#     # img_dist = np.uint8(img_dist)
#     # test = image.bg_subtraction(mode='pos',bg_blur_size=15,fg_blur_size=11)
#     blur = cv.GaussianBlur(image.gray, (5, 5), 0)
#     ret1, th1 = cv.threshold(blur, blur.max()*0.7, 255, cv.THRESH_BINARY)
#     # print ret1
#     # ret3,th3 = cv.threshold(blur,ret1,255,cv.THRESH_BINARY)
#     # print ret3
#     # ret,thresh1 = cv.threshold(blur,equ.max()*0.7,255,cv.THRESH_BINARY)
#     # ret3,th3 = cv.threshold(blur,120,255,cv.THRESH_BINARY)
#     # print ret1
#     edges = cv.Canny(blur, ret1*9/10, ret1)
#     output.publish(edges, 'gray', '/canny_be')
#     # kernel = np.ones((7,7),np.uint8)
#     # edges = cv.dilate(edges,kernel)
#     # output.publish(edges,'gray','/canny_af')
#     # output.publish(ima,'gray','/1')
#     # output.publish(th3,'gray','/th3')
#     # output.publish(blur,'gray','/2')
#     output.publish(th1, 'gray', '/th1')
#     # output.publish(otsu_th,'gray','/otsu')
#     # output.publish(img_dist,'gray','/otsu_dist')
#     # output.publish(img_dist,'gray','/otsu_dist')
#     # output.publish(test,'gray','/test')
#     # cnt = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
#     # edges = cv.bitwise_and(edges,th1)
#     edges = cv.GaussianBlur(edges, (5, 5), 0)
#     kernel = np.ones((1, 21), np.uint8)
#     edges = cv.dilate(edges, kernel)
#     kernel = np.ones((11, 1), np.uint8)
#     edges = cv.dilate(edges, kernel)
#     output.publish(edges, 'gray', '/test')
#     # for i in edges :
#     mask = get_mask()
#     cnt = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
#     # print type(cnt)
#     if len(cnt) > 0:
#         cnt = max(cnt, key=cv.contourArea)
#         rect = cv.minAreaRect(cnt)
#         w_cnt = rect[1][0]
#         h_cnt = rect[1][1]
#         box = cv.boxPoints(rect)
#         box = np.int0(box)
#         approx = cv.approxPolyDP(cnt, 0.025*cv.arcLength(cnt, True), True)
#         print len(approx)
#         area = cv.contourArea(cnt)
#         print area
#         print area/(w_cnt*h_cnt)
#         if area > 3000 and area/(w_cnt*h_cnt) > 0.6:
#             state, cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4 = get_cx(
#                 box, cnt, mode='find_mission')
#             # cv.drawContours(image.display,[box],0,(255,255,255),2)
#             if (state == 1 or func == 'drop') and len(approx) <= 10:
#                 cv.drawContours(image.display, [box], 0, (255, 255, 255), 2)
#                 cv.circle(image.display, (cx4, cy4), 3, (255, 0, 0), -1)
#                 cv.circle(image.display, (cx2, cy2), 3, (0, 255, 0), -1)
#                 cv.circle(image.display, (cx3, cy3), 3, (0, 0, 255), -1)
#                 cv.circle(image.display, (cx1, cy1), 3, (0, 0, 0), -1)
#             elif state == 2 and len(approx) <= 15:
#                 cv.circle(image.display, (cx1, cy1), 3, (255, 255, 255), -1)
#                 cx2 = 0.0
#                 cx3 = 0.0
#                 cx4 = 0.0
#                 cy2 = 0.0
#                 cy3 = 0.0
#                 cy4 = 0.0
#             output.publish(image.display, 'bgr', '/cnt')
#             if func == 'search':
#                 return message(wimg, himg, state, cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4, area/(himg*wimg))
#             else:
#                 return wimg, himg, cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4, edges
#     # cv.drawContours(image.bgr, cnt, -1, (0,0,0), 3)
#     if func == 'search':
#         return message(wimg, himg, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
#     else:
#         return wimg, himg, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0


def find_drop():
    cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4, edges = find_drop_garlic(
        func='drop')
    if cx1 < 0 or cx2 < 0 or cx3 < 0 or cx4 < 0 or cy1 < 0 or cy2 < 0 or cy3 < 0 or cy4 < 0:
        return message(wimg, himg, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    edges = edges[min(int(cy4), int(cy3)):max(int(cy1), int(cy2)), min(
        int(cx1), int(cx4)):max(int(cx2), int(cx3))]
    print cy4
    print cy3
    print cy1
    print cy2
    print cx1
    print cx4
    print cx2
    print cx3
    output.publish(edges, 'gray', '/test2')
    #ret1,th1 = cv.threshold(gray,gray.max()*0.7,255,cv.THRESH_BINARY)
    #th1 = cv.bitwise_not(th1)
    #edges = cv.Canny(blur,ret1*9/10,ret1)
    # output.publish(edges,'gray','/canny_be')
    # output.publish(th1,'gray','/th1_2')
    # gray[gray>40] = 255
    # gray[gray<40] = 0
    gray = cv.bitwise_not(edges)
    cnt = cv.findContours(gray, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    output.publish(gray, 'gray', '/gray_not')
    # print type(cnt)
    if len(cnt) > 0:
        for c in cnt:
            approx = cv.approxPolyDP(c, 0.01*cv.arcLength(c, True), True)
            if len(approx <= 10):
                #cnt = max(cnt,key=cv.contourArea)
                area_slice = cv.contourArea(c)
                rect = cv.minAreaRect(c)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                state, cx1_d, cy1_d, cx2_d, cy2_d, cx3_d, cy3_d, cx4_d, cy4_d = get_cx(
                    box, c, mode='find_slice')
        # print cy1
        # print cy1_d
        # print ('-----')
                cx1 = cx1_d+cx1
                cy1 = cy1-cy1_d
                cx2 = cx1+cx2_d
                cy2 = cy4+cy2_d
                cx3 = cx4+cx3_d
                cy3 = cy1-cy3_d
                cx4 = cx4+cx4_d
                cy4 = cy4+cy4_d
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
                if state == 1:
                    cv.drawContours(
                        image.display, [box], 0, (255, 255, 255), 2)
                    cv.circle(image.display, (cx4, cy4), 3, (255, 0, 0), -1)
                    cv.circle(image.display, (cx2, cy2), 3, (0, 255, 0), -1)
                    cv.circle(image.display, (cx3, cy3), 3, (0, 0, 255), -1)
                    cv.circle(image.display, (cx1, cy1), 3, (0, 0, 0), -1)
                    return message(wimg, himg, state, cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4, area_slice/(himg*wimg))
                elif state == 2:
                    cv.circle(image.display, (int(cx1), int(cy1)),
                              5, (255, 255, 255), -1)
                    return message(wimg, himg, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                output.publish(image.display, 'bgr', '/cnt')
    # cv.drawContours(image.bgr, cnt, -1, (0,0,0), 3)
    return message(wimg, himg, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def find_drop_garlic(func):
    if image.bgr is None:
        output.img_is_none()
        return message(state=-1)
    image.renew_display()

    query_keypoint, query_desc = detector.compute(image.display)
    matches = detector.flann.knnMatch(query_desc, detector.train_desc, k=2)

    good_match = []
    for m, n in matches:
        if(m.distance < 0.80*n.distance):
            good_match.append(m)
    himg, wimg = image.display.shape[:2]

    # print
    cv.putText(image.display, 'Vampire',
               (3, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255),
               2, cv.LINE_AA)

    if(len(good_match) > detector.MIN_MATCH_COUNT):
        tp = []
        qp = []
        for m in good_match:
            tp.append(detector.train_keypoint[m.trainIdx].pt)
            qp.append(query_keypoint[m.queryIdx].pt)
        tp, qp = np.float32((tp, qp))
        H, status = cv.findHomography(tp, qp, cv.RANSAC, 3.0)
        query_border = cv.perspectiveTransform(detector.get_train_border(), H)
        cv.polylines(image.display, [np.int32(
            query_border)], True, (0, 255, 0), 5)
        box = np.int64(query_border[0])

        # print #/89 in display
        detect_point = str(len(good_match))+"/" + \
            str(len(detector.train_keypoint))
        cv.putText(image.display, detect_point,
                   (3, himg-10), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0),
                   2, cv.LINE_AA)
        output.log('FOUND', AnsiCode.GREEN)
        return a_message(state=1, box=box, area=-1, center=False)
    else:
        # print #/89 in display
        detect_point = str(len(good_match))+"/" + \
            str(len(detector.train_keypoint))
        cv.putText(image.display, detect_point,
                   (3, himg-10), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255),
                   2, cv.LINE_AA)
        output.log('NOT FOUND', AnsiCode.RED)
        return a_message()


if __name__ == '__main__':
    rospy.init_node('vision_drop_garlic', anonymous=False)
    output.log("INIT NODE", AnsiCode.GREEN)
    rospy.Subscriber('/vision/bottom/image_raw/compressed',
                     CompressedImage, image.callback)
    output.log("INIT SUBSCRIBER", AnsiCode.GREEN)
    rospy.Service('/vision/drop_garlic',
                  VisionSrvDropGarlic(), mission_callback)
    output.log("INIT SERVICE", AnsiCode.GREEN)
    rospy.spin()
    output.log("END PROGRAM", AnsiCode.YELLOW_HL + AnsiCode.RED)
