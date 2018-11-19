#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
from matplotlib import pyplot as plt
import operator

def radius(x):
   return x[2]

img = cv2.imread('/home/gonzales/Documents/catkin_ws/src/SEMEAR_PTR/img/manometro4.jpg',1)      # trainImage
plt.imshow(img)

img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
img = cv2.inRange(img, (0, 0, 100), (180, 255, 255))
        
# Usa Thresh_Binary para conseguir escala de preto
#ret, img = cv2.threshold(img,110,255,cv2.THRESH_BINARY)

plt.figure("Threshold")
plt.imshow(img,'gray')

# Detecta o Circulo do manometro
circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,minDist=1,
                            param1=30,param2=50,minRadius= 10,maxRadius=0)

if circles is not None:
   print("Nenhum circulo detectado")

   # Corta um circulo interior à esse
   height,width = img.shape
   mask = np.zeros((height,width), np.uint8)

   circles = np.uint16(np.around(circles))
   circle = max(circles[0], key=radius)

   cv2.circle(mask,(circle[0],circle[1]),int(circle[2]),(255,255,255),thickness=-1) # diminui o raio do circulo
   img = cv2.bitwise_and(img, img, mask=mask)

   plt.figure("Cut Circle")
   plt.imshow(img,'gray')

# Usa transformação morfológica CLOSE para manter apenas o ponteiro
kernel = np.ones((4,4),np.uint8)
img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

plt.figure("Close")
plt.imshow(img,'gray')

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

# Ajusta uma reta
line = cv2.fitLine(points=hull, distType=cv2.DIST_L2, param=0, reps=0.01, aeps=0.01)

rows, cols = img.shape
vx = line[0]
vy = line[1]
x = line[2]
y = line[3]
lefty = round( (-x * vy / vx) + y)
righty = round((( cols - x) * vy / vx) + y)
point1 = ( int( cols - 1), int(righty))
point2 = ( int( 0), int(lefty))

cv2.line(img, point1, point2, (127,127,127), 2, cv2.LINE_AA, 0)

plt.figure('Final')
plt.imshow(img, 'gray')
plt.show()

## Ideiass

# É possível localizar o ponteiro  pelo "Aspect Ratio", "Extent",  ou comparando o "Equivalent Diameter" (o diâmetro do circulo equivalente)

# Fit ellipse
#(x,y),(MA,ma),angle = cv2.fitEllipse(cnt)