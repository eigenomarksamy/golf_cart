#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


max_speed = 3.6		#TODO: Find the actual maximum speed
aucmd_topic = '/audi_twist_cmd'
thrtl_topic = '/throttle_cmd'
brake_topic = '/brake_cmd'
steer_topic = '/steering_cmd'
thrtl_pub = rospy.Publisher(thrtl_topic, Float64, queue_size=10)
brake_pub = rospy.Publisher(brake_topic, Float64, queue_size=10)
steer_pub = rospy.Publisher(steer_topic, Float64, queue_size=10)


def twist_callback(twist_msg):
	raw_vel = twist_msg.linear.x
	raw_ang = twist_msg.angular.z
	if raw_vel > max_speed:
		raw_vel = max_speed
	else:
		raw_vel = raw_vel
	if raw_vel > 0:
		thrtl_cmd = raw_vel / max_speed
		brake_cmd = 0.0
	else:
		thrtl_cmd = 0.0
		brake_cmd = 1.0
	steer_cmd = raw_ang		#TODO: 1) Convert angular rate to steering wheel angle & 2) Limit the angle
	thrtl_pub.publish(Float64(thrtl_cmd))
	brake_pub.publish(Float64(brake_cmd))
	steer_pub.publish(Float64(steer_cmd))


def twist_control():
	rospy.init_node('twist_single_audi', anonymous=True)
	rospy.Subscriber(aucmd_topic, Twist, twist_callback)
	thrtl_pub.publish(Float64(0.0))
	brake_pub.publish(Float64(0.0))
	steer_pub.publish(Float64(0.0))
	rospy.spin()


if __name__ == '__main__':
	try:
		twist_control()
	except rospy.ROSInterruptException:
		pass
