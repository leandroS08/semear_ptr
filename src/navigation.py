#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy    

from semear_ptr.srv import Navigation
from semear_ptr.srv import FindPath
from semear_ptr.srv import Cone, ConeResponse

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

import numpy as np 
import cv2
from matplotlib import pyplot as plt
from math import atan2, sqrt

""" Função responsável por virar o robô para o angulo determinado
"""
caminhos_proibidos=[]
odom = None

vertice0 = [6.7960, 4.5500, 90.00   * np.pi/180]
vertice1 = [5.7750, 5.0000, 130.00  * np.pi/180]
vertice2 = [6.2000, 6.3500, 90.00   * np.pi/180]
vertice3 = [5.0500, 6.1250, -160.00 * np.pi/180]
vertice4 = [3.8250, 5.4500, 100.00  * np.pi/180]
vertice5 = [3.6000, 6.6250, 4.9996  * np.pi/180]


vertices = [
vertice0,
vertice1,
vertice2,
vertice3,
vertice4,
vertice5
]
def odom_callback(data):
    global odom
    odom = data

def resetar_odometria():
    pass

def olhar_para(ang):
    global odom

    sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback)

    while( odom is None):
        print("Waiting for Odometry to Publish")
        rospy.Rate(10).sleep()

    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    
    quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
    euler = euler_from_quaternion(quat)
    yaw = euler[2]

    while(abs(ang - yaw) > 0.01 ):
        quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
        yaw = euler_from_quaternion(quat)[2]
        vel_msg.angular.z = (ang - yaw)
        if(vel_msg.angular.z) > 0.6:
            vel_msg.angular.z = 0.6
        elif vel_msg.angular.z < 0.02:
            vel_msg.angular.z = 0.02

        vel_publisher.publish(vel_msg)
        rospy.Rate(20).sleep()

    vel_msg.angular.z = 0
    vel_publisher.publish(vel_msg)

    sub_odom.unregister()
    odom=None

def ir_para(x_goal,y_goal):
    global odom

    sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback)

    while( odom is None):
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
    
   
    x = odom.pose.pose.position.x 
    y = odom.pose.pose.position.y

    while( abs(x-x_goal) > 0.025 or abs(y-y_goal) > 0.025 ):
        
        x = odom.pose.pose.position.x 
        y = odom.pose.pose.position.y

        vel_msg.linear.x = (x-x_goal)
        vel_msg.linear.y = (y-y_goal) 
        
        if vel_msg.linear.x > 0.25:
            vel_msg.linear.x = 0.25
        if vel_msg.linear.y > 0.25:
            vel_msg.linear.y = 0.25
            
        vel_publisher.publish(vel_msg)
        
        rospy.Rate(20).sleep() 

    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_publisher.publish(vel_msg)
    sub_odom.unregister()
    odom=None


def handle_navigation(req):
    global caminhos_proibidos

    chegou = False
    v_atual = req.start
    v_final = req.end
    
    # Olhar para o próximo vertice
    angulo = atan2(vertices[v_final][1] - vertices[v_atual][1],  vertices[v_final][0] - vertices[v_atual][0])
    olhar_para(angulo)
    ir_para( vertices[v][1], vertices[v][0] )
    olhar_para(vertices[v][2])

    return 
    
def navigation_server():
    rospy.init_node('navigation_server')
    s = rospy.Service('navigation', Navigation, handle_navigation)

    print("Ready to Navegate.")
    rospy.spin()


if __name__ == "__main__":
    navigation_server()