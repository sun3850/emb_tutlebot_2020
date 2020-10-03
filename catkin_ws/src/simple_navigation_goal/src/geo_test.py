#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

ori_x = ori_y = ori_z = ori_w = 0.0

def callback(msg):
    global ori_x , ori_y , ori_z , ori_w
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    ori_x, ori_y, ori_z, ori_w  = orientation_list
    print(msg.pose.pose.position)
    print("---------------------")


rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()

