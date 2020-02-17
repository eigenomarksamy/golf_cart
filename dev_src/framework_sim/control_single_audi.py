#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

cmd_throttle = 0.0
cmd_brake = 0.0
cmd_steering = 0.0
max_speed = 3.6
vel_pub = rospy.Publisher('audi_cmd_vel', Twist, queue_size=1000)
vel_msg = Twist()


def callback_throttle(data):
	cmd_throttle = data.data
	print "Throttle", cmd_throttle


def callback_steering(data):
	cmd_steering = data.data
	print "Steer", cmd_steering


def callback_brake(data):
	cmd_brake = data.data
	print "Brake", cmd_brake


def map_cmd():
	rospy.init_node('control_single_audi', anonymous=True)
	rospy.Subscriber("brake_cmd", Float64, callback_brake)
	rospy.Subscriber("steering_cmd", Float64, callback_steering)
	rospy.Subscriber("throttle_cmd", Float64, callback_throttle)
	vel_msg.linear.x = cmd_throttle * max_speed * (1.0 - cmd_brake)
	vel_msg.linear.y = 0.0
	vel_msg.linear.z = 0.0
	vel_msg.angular.x = 0.0
	vel_msg.angular.y = 0.0
	vel_msg.angular.z = cmd_steering
	vel_pub.publish(vel_msg)
	rospy.spin()



def main():
	try:
		map_cmd()
	except rospy.ROSInterruptException:
		pass


if __name__ == '__main__':
	main()