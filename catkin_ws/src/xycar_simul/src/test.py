import sys
import math
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2 as cv
import numpy as np
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy


def region_of_interest(img, vertices, color3=(255,255,255), color1 = 255): #ROI셋팅
    mask = np.zeros_like(img) # img와 같은 크기의 빈 이미지 생성

    if(len(img.shape) > 2): #color 이미지가 3채널이라면
        color = color3
    else: #흑백 이미지 라면
        color = color1

    #vertices에 정의한 점들로 이루어진 ROI설정 부분을 color로 채움
    cv.fillPoly(mask, vertices, color)

    #이미지와 color로 채워진 ROI를 합침
    ROI_img = cv.bitwise_and(img, mask)
    return ROI_img





def mark_img(img, blue_threshold = 110, green_threshold = 110, red_threshold = 110):

    # BGR 제한값 설정
    bgr_threshold = [blue_threshold, green_threshold, red_threshold]

    # BGR 제한값보다 작으면 검은색으로 변경
    thresholds = (src[:, :, 0] < bgr_threshold[0]) \
                 | (src[:, :, 1] < bgr_threshold[1]) \
                 | (src[:, :, 2] < bgr_threshold[2])
    mark[thresholds] = [0, 0, 0]
    return mark


cap = cv.VideoCapture("track-s.mkv")

while(cap.isOpened()):
    ret, src = cap.read()
    src = cv.resize(src, (640, 360))
    dst = cv.Canny(src , 50, 200, None, 3)
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)

    height, width = src.shape[:2] #이미지의 높이와 너비

    #사다리꼴 모양의 Points
    vertices = np.array([[(0,height),(0, height/1.5),
                          (width, height/1.5), (width,height)]], dtype=np.int32)

    roi_img = region_of_interest(src, vertices)  # vertices에 정한 점들 기준으로 ROI 이미지 생성
    mark = np.copy(roi_img)  # roi_img 복사
    mark = mark_img(roi_img)  # 흰색 차선 찾기



    cv.imshow("dst", mark)
    cv.waitKey(1)


