#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
from matplotlib import pyplot as plt
import operator

def nothing(x):
    pass

img = cv2.imread('/home/gonzales/Documents/catkin_ws/src/SEMEAR_PTR/img/manometro3.jpg',0)      # trainImage
img = cv2.medianBlur(img,5)

cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('Low_Thresh','image',1,255,nothing)
cv2.createTrackbar('minDist','image',1,100,nothing)
cv2.createTrackbar('param1','image',1,100,nothing)
cv2.createTrackbar('param2','image',1,100,nothing)
cv2.createTrackbar('minRadius','image',1,100,nothing)
cv2.createTrackbar('maxRadius','image',1,100,nothing)


while(1):

    # get current positions of four trackbars
    low = cv2.getTrackbarPos('Low_Thresh','image')
    minDist = cv2.getTrackbarPos('minDist','image')
    param1 = cv2.getTrackbarPos('param1','image')
    param2 = cv2.getTrackbarPos('param2','image')
    minRadius = cv2.getTrackbarPos('minRadius','image')
    maxRadius = cv2.getTrackbarPos('maxRadius','image')

    ret,th1 = cv2.threshold(img,low,255,cv2.THRESH_BINARY)
    th2 = th1.copy()

    # Detecta o Circulo do manometro
    circles = cv2.HoughCircles(th1,cv2.HOUGH_GRADIENT,1,minDist=len(img)*minDist/100,
                        param1=param1,param2=param2,minRadius=len(img)*minRadius/100, maxRadius=len(img)*maxRadius/100)

    # ensure at least some circles were found
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
    
        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(th1, (x, y), r, (0, 255, 0), 4)

    titles=['original', 'thresh', 'with circles']
    images = [img, th1, th2] 

    for i in range(len(titles)):
        cv2.imshow(titles[i],images[i])

    k = cv2.waitKey(1)
    if k == 27:
        break

# Good size 13 and C = 7