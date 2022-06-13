#! /usr/bin/env python

# Make a python node executable
# chmod u+x ~/catkin_ws/src/beginner_msgsrv/src/projecttask2.py

import rospy
import sys
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from beginner_msgsrv.srv import*
from tf.transformations import euler_from_quaternion

nodeid = str(sys.argv[1])
nodename = 'robot_' + nodeid
rospy.init_node (nodename, anonymous=True)
x = 0
y = 0
theta = 0
def odom(msg):
	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

pub = rospy.Publisher(nodename +'/cmd_vel', Twist, queue_size=10)
sub = rospy.Subscriber(nodename +'/odom', Odometry, odom)

rate = rospy.Rate(10)
goal_position = Point()

#way 1 starts
goal_x = int(sys.argv[2])
goal_y = int(sys.argv[3])

goal_position.x = goal_x
goal_position.y = goal_y	
#way 1 finishes

#way2 starts
	
if int(sys.argv[1]) == 0:
	goal_x = rospy.get_param("goalX_r1")
	goal_y = rospy.get_param("goalY_r1")
	

elif int(sys.argv[1]) == 1:
	goal_x = rospy.get_param("goalX_r2")
	goal_y = rospy.get_param("goalY_r2")

goal_position.x = goal_x
goal_position.y = goal_y
#way2 finishes

#way3 starts


if int(sys.argv[1]) == 0:
	goalX_r1 = 3
	goalY_r1 = 2
	goal_position.x = goalX_r1
	goal_position.y = goalY_r1

elif int(sys.argv[1]) == 1:
	goalX_r2 = 1
	goalY_r2 = 4
	goal_position.x = goalX_r2
	goal_position.y = goalY_r2
#way3 finishes

def euclidean_distance(goal_pose):
	global x
	global y
	rospy.wait_for_service("euclideanDistance")
	euclideanDist = rospy.ServiceProxy("euclideanDistance", euclideandistance)	
	response = euclideanDist(goal_pose.x, x , goal_pose.y, y)
	return response.result
	
def linear_vel(goal_pose, constant=1.5):
	return constant * euclidean_distance(goal_pose)

def steering_angle(goal_pose):
	global x
	global y
	return atan2(goal_pose.y - y, goal_pose.x - x)

def angular_vel(goal_pose, constant=6):
	global theta
	return constant * (steering_angle(goal_pose) - theta)

def move2goal(goal_position):
	tolerance = 0.01

	vel_msg = Twist()
	while euclidean_distance(goal_position) >= tolerance:
		vel_msg.linear.x = linear_vel(goal_position)
		vel_msg.linear.y = 0.0
		vel_msg.linear.z = 0.0

		vel_msg.angular.x = 0.0
		vel_msg.angular.y = 0.0
		vel_msg.angular.z = angular_vel(goal_position)

		pub.publish(vel_msg)

	vel_msg.linear.x = 0.0
	vel_msg.angular.z = 0.0
	pub.publish(vel_msg)
	rospy.spin()

if __name__ =="__main__":
	try:	
		move2goal(goal_position)

	except rospy.ROSInterruptException:
		pass



