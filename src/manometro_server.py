#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy    

from semear_ptr.srv import Manometro
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import numpy as np 
import math
import cv2
import operator
from matplotlib import pyplot as plt

from math import atan2

radianos_manometro_0 = -0.737011559754

img = None
bridge = CvBridge()

def radius(x):
   return x[2]

def image_callback(data):
    global img
    global bridge
    img = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")


def handle_manometro(req):
    global img       
    
    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
        
    while ( img is None):
        print("MANOMETRO - Waiting Image")
        rospy.Rate(10).sleep()
    
    image_sub.unregister()
    
    plt.figure()
    plt.title("Raw")
    plt.imshow(img, 'gray')
   
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img = cv2.inRange(img, (0, 0, 100), (180, 255, 255))

    # Detecta o Circulo do manometro
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,minDist=1,
                                param1=30,param2=50,minRadius= 10,maxRadius=0)

    print(circles)
    if circles is not None:
        # Corta um circulo interior à esse
        height,width = img.shape
        mask = np.zeros((height,width), np.uint8)

        circles = np.uint16(np.around(circles))
        circle = max(circles[0], key=radius)

        cv2.circle(mask,(circle[0],circle[1]),int(circle[2]),(255,255,255),thickness=-1) # diminui o raio do circulo
        img = cv2.bitwise_and(img, img, mask=mask)

    else:
        print("MANOMETRO - Nenhum circulo detectado")

    # Usa transformação morfológica CLOSE para manter apenas o ponteiro
    kernel = np.ones((4,4),np.uint8)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

    # Encontra Contornos
    img, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    areas=[]
    for i in range(len(contours)):
        areas.append(cv2.contourArea(contours[i]))

    # Encontra o contorno de maior área e exclui (era o do manômetro externo)
    try: 
        max_index  = areas.index(max(areas))
    except ValueError:
        plt.figure()
        plt.imshow(img)
        plt.show()
        img=None
        print("nada encontrado")
        return -9999

    del contours[max_index] # this is the outside circle
    
    # Pega o segundo maior contorno = que é o do ponteiro
    cnt  = max(contours, key = cv2.contourArea)
    
    # Desenha o menor polígono convexo possível
    hull = cv2.convexHull(cnt,returnPoints = True)

    # Ajusta uma reta
    line = cv2.fitLine(points=hull, distType=cv2.DIST_L2, param=0, reps=0.01, aeps=0.01)

    # Pega a inclinação da reta
    rows, cols = img.shape
    vx = line[0]
    vy = line[1]
    angulo = atan2(vy,vx)

    # Calcula o centro do polígono
    M = cv2.moments(hull)
    cx_hull = int(M['m10']/M['m00'])
    cy_hull = int(M['m01']/M['m00'])

    # Calcula o centro do contorno
    M = cv2.moments(cnt)
    cx_cnt = int(M['m10']/M['m00'])
    cy_cnt = int(M['m01']/M['m00'])

    posicao_relativa = atan2(cy_cnt-cy_hull, cx_cnt-cx_hull)

    resultado = 0
    if posicao_relativa > 0:
        if angulo > 0:
            resultado = angulo
        if angulo < 0:
            resultado = angulo + np.pi
    if posicao_relativa < 0:
        if angulo < 0:
            resultado = angulo
        if angulo > 0:
            resultado = angulo - np.pi

#--- Desenha o ponto dos centromeros do 
#   cv2.circle(img,(cx_hull,cy_hull),10,(127,127,127),thickness=-1) # diminui o raio do circulo
#   cv2.circle(img,(cx_cnt,cy_cnt),10,(255,255,255),thickness=-1) # diminui o raio do circulo

#---   Desenha o polígono convexo em volta do ponteiro
    cv2.drawContours(img,[hull],0,(0,0,255),2)

#--- Desenha a linha que deve ser a do ponteiro
    x = line[2]
    y = line[3]
    lefty = round( (-x * vy / vx) + y)
    righty = round((( cols - x) * vy / vx) + y)
    point1 = ( int( cols - 1), int(righty))
    point2 = ( int( 0), int(lefty))
    cv2.line(img, point1, point2, (127,127,127), 2, cv2.LINE_AA, 0)

    plt.figure()
    plt.imshow(img, 'gray')
    plt.show(block=False)
    
    resultado = resultado - radianos_manometro_0
    
    img=None
    rospy.Rate(2).sleep()
    plt.close('all')
   
    return resultado


def manometro_server():
    rospy.init_node('manometro_reader_server')
    s = rospy.Service('manometro', Manometro, handle_manometro)
    print("Ready to Read Manometro.")
    rospy.spin()

if __name__ == "__main__":
  #  manometro_server()
    rospy.init_node('manometro_reader_server')
    
    while not rospy.is_shutdown():
        handle_manometro(1)