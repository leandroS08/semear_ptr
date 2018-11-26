#!/usr/bin/env python
# -*- coding: utf-8 -*-
# bacon2508
import rospy
import numpy as np
from sensor_msgs.msg import Image
from pyzbar import pyzbar
import datetime
from cv_bridge import CvBridge
from semear_ptr.srv import Manometro

csv = open("/home/gonzales/barcode.csv", "w")

found_barcode=None
image = None
bridge = CvBridge()

def image_callback(data):
    global image
    global bridge
    image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")


# Função responsável por procurar e lidar com QR_Codes
found_barcode=set()
def handle_QR_code():
    global csv
    global image
    global found_barcode        

    # Procurar e Ler QR_code
    image_sub = rospy.Subscriber('cv_camera/image_raw', Image, image_callback)
    while ( type(image) == type(None)):
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
        csv.write("{},{}\n".format(datetime.datetime.now(),
            barcodeData))
        csv.flush()
        found_barcode.add(barcodeData)
  
    sensor = barcodeData[0]
    print(sensor)
    
    image = None
    barcodes =  []

    return sensor

def go_to(start,end):
    pass

def alinhar_caserna():
    pass

vertice_sensor1 = 1
vertice_sensor2 = 2
vertice_sensor3 = 3

def main_code():
    global csv

    rospy.init_node("main_code")

    # Declaração dos serviçose
    print("Esperando por serviço 'manometro' ")
    rospy.wait_for_service('manometro')
    manometro_srv = rospy.ServiceProxy('manometro', Manometro)
    
    # Ir para Sensor 1 
    go_to(0, vertice_sensor1)
    
    sensor = handle_QR_code()  # Ler o QR Code

    alinhar_caserna() # Alinhar para ler o sensor
    
    # Chamar função para ler sensor
    if(sensor == 'M'):  # Manometer
        leitura = manometro_srv()

        if( leitura == -9999):
            # Código para caso a leitura tenha dado errado
            pass

        if( leitura < 0.2):
            leitura = 0
        csv.write("{},{}\n".format(datetime.datetime.now(),leitura))
        csv.flush()

    elif(sensor == 'P'):  # Panel
        pass

    elif(sensor == 'V'):  # Valve
        pass

    else:
        pass

    # Voltar para o vértice
    pass


if __name__ == "__main__":
    main_code()

