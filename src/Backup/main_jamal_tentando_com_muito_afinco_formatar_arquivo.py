#!/usr/bin/env python
# -*- coding: utf-8 -*-
# bacon2508
import rospy
from sensor_msgs.msg import Image
from pyzbar import pyzbar

from semear_ptr.srv import Manometro
from semear_ptr.srv import Navigation
from semear_ptr.srv import Painel

from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from robotino_msgs.srv import ResetOdometry, ResetOdometryRequest
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

import numpy as np
import datetime

import tf
csv = open("barcode.csv", "w")

found_barcode = None
image = None
bridge = CvBridge()
found_barcode = set()

vertice_qr_code_1 = 1
vertice_sensor_1 = 2
vertice_alinhar_ponto = 3
vertice_qr_code_2 = 4
vertice_sensor_2 = 5
vertice_qr_code_3 = 6
vertice_sensor_3 = 7
vertice_fim = 8

distance_sensor = None
odom = None


def odom_callback(data):
    global odom
    odom = data


def olhar_para(ang):
    global odom

    sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback)

    while(odom is None):
        print("Waiting for Odometry to Publish")
        rospy.Rate(10).sleep()

    vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    euler = euler_from_quaternion(quat)
    yaw = euler[2]

    while(abs(ang - yaw) > 0.01):
        quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        yaw = euler_from_quaternion(quat)[2]
        vel_msg.angular.z = (ang - yaw)
        vel_publisher.publish(vel_msg)
        rospy.Rate(20).sleep()

    vel_msg.angular.z = 0
    vel_publisher.publish(vel_msg)

    sub_odom.unregister()
    odom = None


def distance_callback(data):
    global distance_sensor
    distance_sensor = data.points[4]  # pegar só o do sensor do meio


def alinhar_qr_code():
    global odom
    global distance_sensor
    sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback)

    while(odom is None):
        print("Waiting for Odometry to Publish")
        rospy.Rate(10).sleep()

    vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    # Pega a leitura do sensor analogico de distancia
    sub_sensor = rospy.Subscriber(
        '/analog_readings', PointCloud, distance_callback)
    while(distance_sensor is None):
        print("Esperando sensor de distancia")
        rospy.Rate(10).sleep()

    distance_sensor_prv = distance_sensor

    alinhado = False
    testar_esquerda = True
    while not alinhado:
        quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        euler = euler_from_quaternion(quat)
        yaw = euler[2]

        olhar_para(yaw+0.1)
        quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        yaw = euler_from_quaternion(quat)[2]
        vel_msg.angular.z = (ang - yaw)
        vel_publisher.publish(vel_msg)
        rospy.Rate(20).sleep()
        # Realizar a leitura da diferença
        delta = distance_sensor - distance_sensor_prv
        if(delta < 0):
            testar_esquerda = False
        if(delta > 0):
            olhar_para(yaw-0.1)

    if(testar_esquerda):
        alinhado = False
        while not alinhado:
            quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
            euler = euler_from_quaternion(quat)
            yaw = euler[2]
            olhar_para(yaw-0.1)
            quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
            yaw = euler_from_quaternion(quat)[2]
            vel_msg.angular.z = (ang - yaw)
            vel_publisher.publish(vel_msg)
            rospy.Rate(20).sleep()

            # Realizar a leitura da diferença
            delta = distance_sensor - distance_sensor_prv
            if(delta > 0):
                olhar_para(yaw+0.1)

        vel_msg.angular.z = 0
        vel_publisher.publish(vel_msg)

    sub_sensor.unregister()
    sub_odom.unregister()
    odom = None
    distance_sensor = None


def image_callback(data):
    global image
    global bridge
    image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")




def handle_QR_code():
    global csv
    global image
    global found_barcode

    # Procurar e Ler QR_code
    image_sub = rospy.Subscriber('cv_camera/image_raw', Image, image_callback)
    while (type(image) == type(None)):
        print("Waiting Image")
        rospy.Rate(10).sleep()

    barcodes = pyzbar.decode(image)

    while (len(barcodes) == 0):
        print("Searching for barcode")
        rospy.Rate(3).sleep()
        barcodes = pyzbar.decode(image)

    image_sub.unregister()

    barcode = barcodes[0]
    barcodeData = barcode.data.decode("utf-8")

    # if the barcode text is currently not in our CSV file, write
    # the timestamp + barcode to disk and update the set
    if barcodeData not in found_barcode:
        csv.write("//{}\n".format(datetime.datetime.now())
        csv.write("QR: {}\n".format(barcodeData))
        csv.flush()
        found_barcode.add(barcodeData)

    sensor = barcodeData[0]
    print(sensor)

    image = None
    barcodes = []

    return sensor


def alinhar_caserna():
    pass


def resetar_odometria():
    odometry_res_srv = rospy.ServiceProxy('reset_odometry', Painel)
    reset_odom_msg = ResetOdometryRequest()

    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/AMR', '/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    reset_odom_msg.phi = euler_from_quaternion(rot)[2]
    reset_odom_msg.x = trans[0]
    reset_odom_msg.y = trans[1]

def main_code():
    global csv
    csv.write("//{}\n".format(datetime.datetime.now())
    csv.write("//ARQUIVO DE LEITURAS - SEMEAR USP\n")
    csv.flush()

    rospy.init_node("main_code")

    # Declaração dos serviços
    print("Esperando por serviço 'manometro' ")
    rospy.wait_for_service('manometro')
    manometro_srv = rospy.ServiceProxy('manometro', Manometro)

    print("Esperando por serviço 'navigation' ")
    rospy.wait_for_service('navigation')
    navigation_srv = rospy.ServiceProxy('navigation', Navigation)

    print("Esperando por serviço do painel ")
    rospy.wait_for_service('painel_vision')
    painel_srv = rospy.ServiceProxy('painel_vision', Painel)
    
    print("Esperando por serviço 'Valvula' ")
    rospy.wait_for_service('valvula_vision')
    valvula_srv = rospy.ServiceProxy('valvula_vision', Painel)

    print("Esperando por serviço 'reset_odometry' ")
    rospy.wait_for_service('reset_odometry')
    
    # Qr_code 1
    navigation_srv.call(0, vertice_qr_code_1)
    alinhar_qr_code()
    sensor = handle_QR_code()  # Ler o QR Code

    # Ir para Sensor 1
    navigation_srv.call(vertice_qr_code_1, vertice_sensor_1)
    alinhar_qr_code()

    # Chamar função para ler sensor
    if(sensor == 'M'):  # Manometer
        leitura = manometro_srv()

        if(leitura == -9999):
            # Código para caso a leitura tenha dado errado
            pass

        if(leitura < 0.2):
            leitura = 0
        
        csv.write("//{}\n".format(datetime.datetime.now())
        csv.write("M: {} graus\n".format(leitura))
        csv.flush()

    elif(sensor == 'P'):  # Panel
        leitura = painel_srv()

        if (leitura.botao1 == True)
            string_botao1 = L
        else
            string_botao1 = D

        if (leitura.botao2 == True)
            string_botao2 = L
        else
            string_botao2 = D

        if (leitura.botao3 == True)
            string_botao3 = L
        else
            string_botao3 = D


        csv.write("//{}\n".format(datetime.datetime.now())
        csv.write("P: {}{}{}\n".format(string_botao1, string_botao2, string_botao3))
        csv.flush()

    elif(sensor == 'V'):  # Valve
        leitura =  valvula_srv()

        if(leitura == True)
            string_valvula = A
        else
            string_valvula = F

        csv.write("//{}\n".format(datetime.datetime.now())
        csv.write("V: {}\n".format(string_valvula))
        csv.flush()
    else:
        pass

    # Resetar odometria
    navigation_srv.call(vertice_sensor_1 , vertice_alinhar_ponto)
    
    # Qr_code 2
    navigation_srv.call(vertice_alinhar_ponto, vertice_qr_code_2)
    alinhar_qr_code()
    sensor = handle_QR_code()  # Ler o QR Code

    # Ir para Sensor 2
    navigation_srv.call(vertice_qr_code_2, vertice_sensor_2)
    alinhar_qr_code()

    # Chamar função para ler sensor
    if(sensor == 'M'):  # Manometer
        leitura = manometro_srv()

        if(leitura == -9999):
            # Código para caso a leitura tenha dado errado
            pass

        if(leitura < 0.2):
            leitura = 0
        
        csv.write("{},{}\n".format(datetime.datetime.now(), leitura))
        csv.flush()

    elif(sensor == 'P'):  # Panel
        leitura = painel_srv()
        csv.write("{}, Botao 1: {}, Botao 2: {}, Botao3: {} \n".format(datetime.datetime.now(), leitura.botao1, leitura.botao2, leitura.botao3))

    elif(sensor == 'V'):  # Valve
        leitura =  valvula_srv()

        csv.write("{},{}\n".format(datetime.datetime.now(), leitura))
        csv.flush()
    else:
        pass

    # Qr_code 3
    navigation_srv.call(vertice_sensor_2, vertice_qr_code_3)
    alinhar_qr_code()
    sensor = handle_QR_code()  # Ler o QR Code

    # Ir para Sensor 3
    navigation_srv.call(vertice_qr_code_3, vertice_sensor_3)
    alinhar_qr_code()

    # Chamar função para ler sensor
    if(sensor == 'M'):  # Manometer
        leitura = manometro_srv()

        if(leitura == -9999):
            # Código para caso a leitura tenha dado errado
            pass

        if(leitura < 0.2):
            leitura = 0
        
        csv.write("{},{}\n".format(datetime.datetime.now(), leitura))
        csv.flush()

    elif(sensor == 'P'):  # Panel
        leitura = painel_srv()
        csv.write("{}, Botao 1: {}, Botao 2: {}, Botao3: {} \n".format(datetime.datetime.now(), leitura.botao1, leitura.botao2, leitura.botao3))

    elif(sensor == 'V'):  # Valve
        leitura =  valvula_srv()

        csv.write("{},{}\n".format(datetime.datetime.now(), leitura))
        csv.flush()
    else:
        pass

    navigation_srv.call(vertice_qr_code_3, vertice_fim)

if __name__ == "__main__":
    main_code()
