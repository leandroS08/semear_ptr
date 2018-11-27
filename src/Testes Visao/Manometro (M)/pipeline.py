#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
from matplotlib import pyplot as plt
import operator

img = cv2.imread('/home/gonzales/Documents/catkin_ws/src/SEMEAR_PTR/img/manometro.jpg',0)      # trainImage

plt.imshow(img)

# Usa Thresh_Binary para conseguir escala de preto
ret, img = cv2.threshold(img,64,255,cv2.THRESH_BINARY)

plt.figure()
plt.imshow(img)

# Detecta o Circulo do manometro
circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,minDist=len(img)/4,
                            param1=50,param2=40,minRadius=len(img)/4,maxRadius=0)

# Corta um circulo interior à esse
height,width = img.shape
mask = np.zeros((height,width), np.uint8)

circles = np.uint16(np.around(circles))
circle = circles[0][0]

cv2.circle(mask,(circle[0],circle[1]),int(circle[2]*0.7),(255,255,255),thickness=-1) # diminui o raio do circulo
img = cv2.bitwise_and(img, img, mask=mask)

# Usa transformação morfológica CLOSE para manter apenas o ponteiro
kernel = np.ones((6,6),np.uint8)
img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

# Encontra Contornos
img, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

areas=[]
for i in range(len(contours)):
   areas.append(cv2.contourArea(contours[i]))

# Encontra o contorno de maior área e exclui (era o do manômetro externo)
max_index  = areas.index(max(areas))
del contours[max_index] # this is the outside circle
cnt  = max(contours, key = cv2.contourArea)

# Desenha o menor polígono convexo possível
hull = cv2.convexHull(cnt,returnPoints = True)
cv2.drawContours(img,[hull],0,(0,0,255),2)

# Ajusta uma reta no polígono
cv.FitLine
plt.figure()
plt.imshow(img, 'gray')
plt.show()




