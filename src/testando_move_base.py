#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rospy import Time
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import numpy as np

if __name__ == "__main__":
    
    rospy.init_node("Testing_Movements")
    
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    rospy.Rate(10).sleep()

    pose = PoseStamped()
    
    pose.header.seq = 1
    pose.header.frame_id = 'map'
    
    Time.now() # The first returns 0
    now = Time.now()
    
    print(now.secs)
    pose.header.stamp.secs = now.secs
    pose.header.stamp.nsecs = now.nsecs

    pose.pose.position.x = 6.8
    pose.pose.position.y = 6
    pose.pose.position.z = 0
    
    roll = 0
    pitch = 0
    yaw = np.pi / 2
    quart = quaternion_from_euler(roll, pitch,yaw)
    pose.pose.orientation.x = quart[0]
    pose.pose.orientation.y = quart[1]
    pose.pose.orientation.z = quart[2]
    pose.pose.orientation.w = quart[3]

    pub.publish(pose)
    
    print(pose)

    for i in range(10):
        
        rospy.Rate(10).sleep()