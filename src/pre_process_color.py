#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt

"""  Basic Color filter 
frame = cv2.imread('/home/gonzales/Documents/catkin_ws/src/SEMEAR_PTR/img/ponteiro.jpg',1)  # queryImage


# Convert BGR to HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# define range of blue color in HSV
lower_blue = np.array([0,0,50])
upper_blue = np.array([359,40,100])

# Threshold the HSV image to get only blue colors
mask = cv2.inRange(hsv, lower_blue, upper_blue)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(frame,frame, mask= mask)

cv2.imshow('frame',frame)
cv2.imshow('mask',mask)
cv2.imshow('res',res)

cv2.waitKey()

cv2.destroyAllWindows()
"""
def nothing(x):
    pass

img = cv2.imread('/home/gonzales/Documents/catkin_ws/src/SEMEAR_PTR/img/manometro.jpg',0)
img = cv2.medianBlur(img,5)

cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('Size','image',1,20,nothing)
cv2.createTrackbar('C','image',1,20,nothing)


while(1):

    # get current positions of four trackbars
    r = cv2.getTrackbarPos('Size','image')*2 + 1
    g = cv2.getTrackbarPos('C','image')
    
    print(r)
    print(g)
    if r < 3 :
        r=3
    if g < 1 :
        g = 1
    ret,th1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)

    th2 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
                cv2.THRESH_BINARY,r,g)
    
    th3 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY,r,g)

    titles = ['Original Image', 'Global Thresholding (v = 127)',
                'Adaptive Mean Thresholding', 'Adaptive Gaussian Thresholding']
    images = [img, th1, th2, th3]

    for i in xrange(4):
        cv2.imshow(titles[i],images[i])
    
    k = cv2.waitKey(1)
    if k == 27:
        break

# Good size 13 and C = 7