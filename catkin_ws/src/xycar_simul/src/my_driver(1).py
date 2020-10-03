#!/usr/bin/env python
# -*- coding: cp949 -*-
# -*- coding: utf-8 -*- # 한글 주석쓰려면 이거 해야함
import rospy
from std_msgs.msg import Int32MultiArray
import sys
import math
import cv2
import numpy as np

import time


def pub_motor(Angle, Speed):
    drive_info = [Angle, Speed]
    drive_info = Int32MultiArray(data = drive_info)
    pub.publish(drive_info)

def grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)


def canny(img, low_threshold, high_threshold):
    return cv2.Canny(img, low_threshold, high_threshold)


def gaussian_blur(img, kernel_size):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


def region_of_interest(img, vertices, color3=(255, 255, 255), color1=255):

    mask = np.zeros_like(img)  #

    if len(img.shape) > 2:
        color = color3
    else:
        color = color1


    cv2.fillPoly(mask, vertices, color)


    ROI_image = cv2.bitwise_and(img, mask)
    return ROI_image


def draw_lines(img, lines, color=[0, 0, 255], thickness=2):  #

    cv2.line(img, (lines[0][0][0], lines[0][0][1]), (lines[0][0][2], lines[0][0][3]), color, thickness)


def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):  #
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len,
                            maxLineGap=max_line_gap)
    #print(lines)
    #print(len(lines))
    #print(type(lines))
    #print("-----------------------")
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)

    if (float(lines[0][0][1] - lines[0][0][3]) == 0):
        degree = 0
    else:
        degree = (float((lines[0][0][0] - lines[0][0][2]) / (lines[0][0][1] - lines[0][0][3])))

    return line_img, degree


def weighted_img(img, initial_img, a=1, b=1., c=0.):
    return cv2.addWeighted(initial_img, a, img, b, c)



def start():
    global pub
    rospy.init_node('my_driver')
    pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
    rate = rospy.Rate(30)

    Angle = 0
    Speed = 20

    cap = cv2.VideoCapture('/home/hyeon/catkin_ws/src/xycar_simul/track-s.mkv')  #

    while (cap.isOpened()):
        ret, image = cap.read()

        if ret == False:
            break


        height, width = image.shape[:2]  #

        gray_img = grayscale(image)  #

        blur_img = gaussian_blur(gray_img, 3)  #

        canny_img = canny(blur_img, 70, 210)  #

        vertices = np.array([[(0, height), (0, height / 1.5),
                              (width, height / 1.5), (width, height)]], dtype=np.int32)
        ROI_img = region_of_interest(canny_img, vertices)  #

        hough_img, degree = hough_lines(ROI_img, 1, 1 * np.pi / 180, 30, 10, 20)

        result = weighted_img(hough_img, image)  #
        cv2.imshow('result', result)  #
        cv2.waitKey(1)

        if(int(degree) < 0 ):
            Speed = 20
            Angle = 100

        if(int(degree) > 0):
            Speed = 20
            Angle = -100


        pub_motor(Angle, Speed)
        rate.sleep()


if __name__ == '__main__':

    start()
