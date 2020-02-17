#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


aucmd_topic = '/orange/cmd_vel'
steer_topic = '/orange/steering_cmd'
steer_pub = rospy.Publisher(steer_topic, Float64, queue_size=10)


def twist_callback(twist_msg):
	raw_ang = twist_msg.angular.z
	steer_cmd = raw_ang		#TODO: 1) Convert angular rate to steering wheel angle & 2) Limit the angle
	steer_pub.publish(Float64(steer_cmd))


def twist_lat_control():
	rospy.init_node('twist_orange_audi', anonymous=True)
	rospy.Subscriber(aucmd_topic, Twist, twist_callback)
	steer_pub.publish(Float64(0.0))
	rospy.spin()


if __name__ == '__main__':
	try:
		twist_lat_control()
	except rospy.ROSInterruptException:
		pass
