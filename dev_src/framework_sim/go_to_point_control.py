#! /usr/bin/env python

import math
import time
import rospy
import tf
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped


def gt_callback(groundtruth):
	x = groundtruth.twist.linear.x
	y = groundtruth.twist.linear.y
	yaw = groundtruth.twist.angular.z
	feedback = [x, y, yaw]
	x_vel, yaw_rate = controller(feedback)
	control_msg.linear.x = x_vel
	control_msg.angular.z = yaw_rate
	control_pub.publish(control_msg)


def odom_callback(odom):
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]
	yaw = odom.pose.pose.orientation.z
	feedback = [x, y, yaw]
	x_vel, yaw_rate = controller(feedback)
	control_msg.linear.x = x_vel
	control_msg.angular.z = yaw_rate
	control_pub.publish(control_msg)


def controller(feedback):
	x_vel = 0.0
	yaw_rate = 0.0
	x = feedback[0]
	y = feedback[1]
	yaw = feedback[2]
	dist_err = calc_distance_error(x, y)
	ang_err = calc_angle_error(x, y)
	x_vel = calc_req_vel(dist_err)
	yaw_rate = calc_req_rate(ang_err)
	return x_vel, yaw_rate


def calc_distance_error(x, y):
	return (((x_goal - x) ** 2 + (y_goal - y) ** 2) ** 0.5)


def calc_angle_error(x, y):
	# print math.atan2(y_goal - y, x_goal - x)
	return (math.atan2(y_goal - y, x_goal - x))


def calc_req_vel(dist_err):
	if dist_err >= 0.7:
		var = (1.0 * dist_err)
	else:
		var = 0.0
	return var


def calc_req_rate(ang_err):
	return (1.0 * ang_err)


def get_goal():
	x_goal_loc = input("Desired X: ")
	y_goal_loc = input("Desired Y: ")
	goal_loc = [x_goal_loc, y_goal_loc]
	return goal_loc


def main():
	global goal, x_goal, y_goal, control_topic, control_pub, control_msg
	x_goal = 0
	y_goal = 0
	yaw_goal = 0
	node_name = 'goto_single_audi'
	gt_topic = '/twist'
	odom_topic = '/audi_odom'
	control_topic = 'audi_twist_cmd'
	init_pub = rospy.Publisher(control_topic, Twist, queue_size=10)
	control_pub = rospy.Publisher(control_topic, Twist, queue_size=10)
	init_msg = Twist()
	control_msg = init_msg
	init_msg.linear.x = 0.0
	init_msg.linear.y = 0.0
	init_msg.linear.z = 0.0
	init_msg.angular.x = 0.0
	init_msg.angular.y = 0.0
	init_msg.angular.z = 0.0
	goal = get_goal()
	x_goal = goal[0]
	y_goal = goal[1]
	goalie()
	try:
		rospy.init_node(node_name, anonymous=True)
		# rospy.Subscriber(gt_topic, TwistStamped, gt_callback)
		rospy.Subscriber(odom_topic, Odometry, odom_callback)
		init_pub.publish(init_msg)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


def goalie():
	print goal
	print x_goal
	print y_goal


if __name__ == '__main__':
	main()