#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
from matplotlib import pyplot as plt
import operator

def nothing(x):
    pass

img = cv2.imread('/home/gonzales/Documents/catkin_ws/src/SEMEAR_PTR/img/manometro.jpg',0)      # trainImage
img = cv2.medianBlur(img,5)

cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('Low_Thresh','image',1,255,nothing)
cv2.createTrackbar('Kernel Size','image',3,20,nothing)


while(1):

   # get current positions of four trackbars
   low = cv2.getTrackbarPos('Low_Thresh','image')
   ksize = cv2.getTrackbarPos('Kernel Size','image')
   
   
   ret,th1 = cv2.threshold(img,low,255,cv2.THRESH_BINARY)
   # Open
   kernel = np.ones((ksize,ksize),np.uint8)
   # Usa transformação morfológica CLOSE para manter apenas o ponteiro
   th2 = cv2.morphologyEx(th1, cv2.MORPH_CLOSE, kernel)
   th3 = cv2.morphologyEx(th2, cv2.MORPH_CLOSE, kernel)

   titles = ['Original Image', 'Thresholding (v = 127)',
               'Close1', 'Close2']
   images = [img, th1, th2, th3]

   for i in range(4):
      cv2.imshow(titles[i],images[i])
   
   k = cv2.waitKey(1)
   if k == 27:
      break

# Good size 13 and C = 7