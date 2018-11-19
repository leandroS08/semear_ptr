#!/usr/bin/env python

import numpy as np
import cv2
from matplotlib import pyplot as plt


img1 = cv2.imread('/home/gonzales/Documents/catkin_ws/src/SEMEAR_PTR/img/ponteiro.jpg',0)          # queryImage
img2 = cv2.imread('/home/gonzales/Documents/catkin_ws/src/SEMEAR_PTR/img/manometro.jpg',0)      # trainImage
img3 = cv2.imread('/home/gonzales/Documents/catkin_ws/src/SEMEAR_PTR/img/marca.jpeg',0)      # trainImage

img1 = cv2.adaptiveThreshold(img1,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY,13,7 )
img2 = cv2.adaptiveThreshold(img2,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY,13,7 )

# Initiate ORB detector
orb = cv2.ORB_create()

# find the keypoints and descriptors with ORB
kp1, des1 = orb.detectAndCompute(img1,None)
kp2, des2 = orb.detectAndCompute(img2,None)

# create BFMatcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Match descriptors.
matches = bf.match(des1,des2)

# Sort them in the order of their distance.
matches = sorted(matches, key = lambda x:x.distance)
print(matches)

# Draw first 10 matches.
img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:20], flags=2,outImg=img2)


plt.imshow(img3),plt.show()