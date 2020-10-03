# -*- coding: cp949 -*-
# -*- coding: utf-8 -*- # �ѱ� �ּ������� �̰� �ؾ���
import sys
import math
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
import numpy as np
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy



def grayscale(img):  # ����̹����� ��ȯ
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)


def canny(img, low_threshold, high_threshold):  # Canny �˰���
    return cv2.Canny(img, low_threshold, high_threshold)


def gaussian_blur(img, kernel_size):  # ����þ� ����
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


def region_of_interest(img, vertices, color3=(255, 255, 255), color1=255):  # ROI ����

    mask = np.zeros_like(img)  # mask = img�� ���� ũ���� �� �̹���

    if len(img.shape) > 2:  # Color �̹���(3ä��)��� :
        color = color3
    else:  # ��� �̹���(1ä��)��� :
        color = color1

    # vertices�� ���� ����� �̷��� �ٰ����κ�(ROI �����κ�)�� color�� ä��
    cv2.fillPoly(mask, vertices, color)

    # �̹����� color�� ä���� ROI�� ��ħ
    ROI_image = cv2.bitwise_and(img, mask)
    return ROI_image


def draw_lines(img, lines, color=[0, 0, 255], thickness=2):  # �� �׸���

    cv2.line(img, (lines[0][0][0], lines[0][0][1]), (lines[0][0][2], lines[0][0][3]), color, thickness)


def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):  # ���� ��ȯ
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len,
                            maxLineGap=max_line_gap)
    print(lines)
    print(len(lines))
    print(type(lines))
    print("-----------------------")
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)

    print(float((lines[0][0][0] - lines[0][0][2])/ (lines[0][0][1] - lines[0][0][3])))
    degree = 0

    return line_img, degree


def weighted_img(img, initial_img, ��=1, ��=1., ��=0.):  # �� �̹��� operlap �ϱ�
    return cv2.addWeighted(initial_img, ��, img, ��, ��)


cap = cv2.VideoCapture('track-s.mkv')  # ������ �ҷ�����

while (cap.isOpened()):
    ret, image = cap.read()

    height, width = image.shape[:2]  # �̹��� ����, �ʺ�

    gray_img = grayscale(image)  # ����̹����� ��ȯ

    blur_img = gaussian_blur(gray_img, 3)  # Blur ȿ��

    canny_img = canny(blur_img, 70, 210)  # Canny edge �˰���

    vertices = np.array([[(0,height),(0, height/1.5),
                          (width, height/1.5), (width,height)]], dtype=np.int32)
    ROI_img = region_of_interest(canny_img, vertices)  # ROI ����

    hough_img, degree  = hough_lines(ROI_img, 1, 1 * np.pi / 180, 30, 10, 20)  # ���� ��ȯ

    result = weighted_img(hough_img, image)  # ���� �̹����� ����� �� overlap
    cv2.imshow('result', result)  # ��� �̹��� ���
    cv2.waitKey(10)
