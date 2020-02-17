#! /usr/bin/python


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


class GoToPointGoal:

	def calc_velocity(self):
		velocity = 6.0 * self.calc_distance()
		return velocity


	def calc_rotarate(self):
		angular_rate = 9.0 * self.calc_rotation()
		return angular_rate


	def calc_rotation(self):
		angle = math.atan2(self._y_goal - self._y, self._x_goal - self._x)
		return angle


	def calc_distance(self):
		distance = (((self._x_goal - self._x)**2) + ((self._y_goal - self._y)**2))**0.5
		return distance


	def feedback_callback(self, feedback):
		is_feedback_ok = False
		self._is_send_control = False
		quaternion = (feedback.pose.pose.orientation.x, feedback.pose.pose.orientation.y, feedback.pose.pose.orientation.z, feedback.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		yaw = euler[2]
		x = feedback.pose.pose.position.x
		y = feedback.pose.pose.position.y
		psi = yaw
		xd = feedback.twist.twist.linear.x
		psid = feedback.twist.twist.angular.z
		self._x = x
		self._y = y
		self._psi = psi
		self._xd = xd
		self._psid = psid
		is_feedback_ok = True
		control_msg = Twist()
		while self.calc_distance() >= 0.7:		
			velocity = self.calc_velocity()
			rotation = self.calc_rotation()
			rotarate = self.calc_rotarate()
		else:
			velcoity = 0.0
			rotarate = 0.0
		control_msg.linear.x = velocity
		control_msg.angular.z = rotarate
		self._is_send_control = True
		self._control_msg = control_msg
		return is_feedback_ok


	def __init__(self, node_name, model_name, x_pos, y_pos, psi_ori, x_vel, psi_rat, goal):
		self._node_name = node_name
		self._model_name = model_name
		self._x = x_pos
		self._y = y_pos
		self._psi = psi_ori
		self._xd = x_vel
		self._psid = psi_rat
		self._x_goal = goal[0]
		self._y_goal = goal[1]
		self._psi_goal = goal[2]
		self._is_send_control = False
		self._control_msg = Twist()
		# self._feedback_msg = Odometry()
		# self._groundtruth_msg = TwistStamped()

		rospy.init_node(self._node_name, anonymous=True)
		# self._rate = rospy.Rate(10)

		# rospy.Subscriber("/twist", TwistStamped, self.groundtruth_callback)
		rospy.Subscriber("/audi_odom", Odometry, self.feedback_callback)
		control_pub = rospy.Publisher('/audi_twist_cmd', Twist, queue_size=10)
		while not rospy.is_shutdown():
			if self._is_send_control:
				control_pub.publish(self._control_msg)
		rospy.spin()
