#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class CmdMap:

	def call_throttle(data):
		self.throttle_cmd = data
		if self.brake_cmd == 0:
			self.vel_msg.linear.x = self.throttle_cmd
		else:
			self.vel_msg.linear.x = 0.0
		self.send_throttle_ = True

	def call_brake(data):
		self.brake_cmd = data
		self.send_brake_ = True

	def call_steering(data):
		self.steering_cmd = data
		self.vel_msg.angular.z = self.steering_cmd
		self.send_steering_ = True

	def __init__(self):
		self.vel_msg = Twist()
		self.send_throttle_ = False
		self.send_steering_ = False
		self.send_brake_ = False
		rospy.Subscriber("/throttle_cmd", Float64, self.call_throttle)
		rospy.Subscriber("/brake_cmd", Float64, self.call_brake)
		rospy.Subscriber("/steering_cmd", Float64, self.call_steering)
		vel_pub = rospy.Publisher('/audi_cmd_vel', Twist, queue_size=10)
		while not rospy.is_shutdown():
			vel_pub.publish(self.vel_msg)


if __name__ == '__main__':
	rospy.init_node('cmd_map', anonymous=True)
	try:
		cmd_map = CmdMap()
	except rospy.ROSInterruptException:  pass
