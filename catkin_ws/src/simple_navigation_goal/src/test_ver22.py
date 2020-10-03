#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32MultiArray, String
import time
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import math

LINEAR = 0.1

roll = pitch = yaw = 0.0
current_degree = 0
position_x = 0	
position_y = 0ros
kp= 1.5




#Lider값 가공
def callback_lider(scan):
    Front = scan.ranges[0:14] + scan.ranges[345:359]
    Left1 = scan.ranges[15:34]
    Left2 = scan.ranges[35:54]
    Left3 = scan.ranges[55:74]
    Left4 = scan.ranges[75:94]
    Right1 = scan.ranges[325:344]
    Right2 = scan.ranges[305:324]
    Right3 = scan.ranges[285:304]
    Right4 = scan.ranges[265:284]
    back = scan.ranges[95:264]

    #average
    Front_avg = avge(Front)
    Left1_avg = avge(Left1)
    Left2_avg = avge(Left2)
    Left3_avg = avge(Left3)
    Left4_avg = avge(Left4)
    Right1_avg = avge(Right1)
    Right2_avg = avge(Right2)
    Right3_avg = avge(Right3)
    Right4_avg = avge(Right4)
    back_avg = avge(back)

    avg = [Front_avg, Left1_avg, Left2_avg, Left3_avg, Left4_avg, Right1_avg, Right2_avg, Right3_avg, Right4_avg, back_avg]

    return avg;

#lider값 평균 구하기
def avge(arr):
    l = []
    for i in arr:
        if i == float("inf"):
            continue
        elif i == 0:
            continue
        else:
            l.append(i)
    #print(l)
    return sum(l)/(len(l)+0.01)

def move(linear, angular):
	twist = Twist()
	twist.linear.x = linear
	twist.angular.z = angular

	ack_publisher.publish(twist)

def callback(msg):
	global roll, pitch, yaw, position_x, position_y
	orientation_q = msg.pose.pose.orientation
	position_x = msg.pose.pose.position.x
	position_y = msg.pose.pose.position.y
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	print(position_x)


def move_go(dis):
	global position_x, position_y
	for i in range(dis):
		current_position_x = position_x
		current_position_y = position_y
		if(current_degree == 0):
			while not rospy.is_shutdown():
				move(0.02, 0)
				if(int(position_x * 100) == int((current_position_x + 0.1) * 100)):
					move_stop()
					break
		elif(current_degree == -90):
			while not rospy.is_shutdown():
				move(0.02, 0)
				if(int(position_y * 100) == int((current_position_y - 0.1) * 100)):
					move_stop()
					break
		elif(current_degree == 90):
			while not rospy.is_shutdown():
				move(0.02, 0)
				if(int(position_y * 100) == int((current_position_y + 0.1) * 100)):
					move_stop()
					break
		elif(current_degree == 180):
			while not rospy.is_shutdown():
				move(0.02, 0)
				if(int(position_x * 100) == int((current_position_x - 0.1) * 100)):
					move_stop()
					break



def move_front(dis): #정면으로  회전
	global current_degree

	current_degree = 0

	while not rospy.is_shutdown():
		# quat = quaternion_from_euler (roll, pitch,yaw)
		# print quat
		target_rad = 0 * math.pi / 180
		command.angular.z = kp * (target_rad - yaw)
		pub.publish(command)
		print("taeget={} current:{}", current_degree, yaw)
		print(command.angular.z)
		r.sleep()
		if (int(command.angular.z * 100) == 0):
			break
	move_go(dis)


def move_back(dis): #180 방향으로 회전

	global current_degree
	current_degree = 180

	while not rospy.is_shutdown():
		# quat = quaternion_from_euler (roll, pitch,yaw)
		# print quat
		target_rad = 180 * math.pi / 180
		command.angular.z = kp * (target_rad - yaw)
		pub.publish(command)
		print("taeget={} current:{}", current_degree, yaw)
		print(command.angular.z)
		r.sleep()
		if (int(command.angular.z * 100) == 0):
			break
	move_go(dis)

def move_left(dis): #90도 방향으로 회전
	global current_degree
	current_degree =  90
	while not rospy.is_shutdown():
		# quat = quaternion_from_euler (roll, pitch,yaw)
		# print quat
		target_rad = 90 * math.pi / 180
		command.angular.z = kp * (target_rad - yaw)
		pub.publish(command)
		print("taeget={} current:{}", current_degree, yaw)
		print(command.angular.z)
		r.sleep()
		if (int(command.angular.z * 100) == 0):
			break
	move_go(dis)



def move_right(dis): # -90도 방향으로 회전

	global current_degree
	current_degree =  -90

	while not rospy.is_shutdown():
		# quat = quaternion_from_euler (roll, pitch,yaw)
		# print quat
		target_rad = -90 * math.pi / 180
		command.angular.z = kp * (target_rad - yaw)
		pub.publish(command)
		print("taeget={} current:{}", current_degree , yaw)
		print(command.angular.z)
		r.sleep()
		if (int(command.angular.z * 100) == 0):
			break
	move_go(dis)

def move_stop(): #정지
	move(0,0)


rospy.init_node('rotate_robot')

sub = rospy.Subscriber ('/odom', Odometry  , callback)
ack_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
r = rospy.Rate(50)
command =Twist()
time.sleep(2)


#[ERROR] [1600248083.752708]: Message type std_msgs/String does not have a field load


