#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan, PointCloud
#from geometry_msgs.msg import Point32
from semear_ptr.srv import HokuyoObstacleAvoidance, HokuyoObstacleAvoidanceResponse, HokuyoObstacleAvoidanceRequest

from math import sin, cos
import numpy as np
#from sklearn.cluster import KMeans

hokuyo_msg = LaserScan()

# pub = rospy.Publisher('/debug', PointCloud, queue_size=10)

def callback(data):
    global hokuyo_msg
    hokuyo_msg = data
    
# Só funciona para quando o robô está em paralelo ou ortogonal com as paredes

def handle_hokuyo_obstacle_avoidance(req):
    global hokuyo_msg

    # Distance from the robot to the walls 
    distance_left  = req.distance_wall_left  * 0.9
    distance_right = req.distance_wall_right * 0.9
    distance_front = req.distance_wall_front * 0.9

    angle = hokuyo_msg.angle_min - hokuyo_msg.angle_increment
    increment =  hokuyo_msg.angle_increment
    obstacle_x = []
    obstacle_y = []
    obstacle=[]
#    debug_msg = PointCloud()
#    debug_msg.header = hokuyo_msg.header
    
    for ranges in hokuyo_msg.ranges:
        angle += increment

        x = abs( ranges * cos(angle) )
        if(x > distance_front):
            continue
    
        y = abs(ranges * sin(angle) )

        if(angle > 0 and y > distance_left):
            continue
        elif( angle < 0 and y > distance_right):
            continue
        
        obstacle_x.append(x)
        obstacle_y.append(y)
        obstacle.append([x, y])
    
     #   point = Point32()
     #   point.x = x
     #   point.y = y
     #   point.z = 0
     #   debug_msg.points.append(point)
    
    response = HokuyoObstacleAvoidanceResponse()
    response.obstacle_x = obstacle_x
    response.obstacle_y = obstacle_y
    
    return response

def hokuyo_obstacle_avoidance_server():
    
    rospy.init_node('hokuyo_obstacle')
    
    s = rospy.Service('hokuyo_obstacle', HokuyoObstacleAvoidance, handle_hokuyo_obstacle_avoidance)
    rospy.Subscriber("/scan", LaserScan, callback)
    
    print("Read to use Hokuyo to Avoid Obstacles")
    
    rospy.spin()


if __name__ == "__main__":
    hokuyo_obstacle_avoidance_server()