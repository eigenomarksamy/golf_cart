#! /usr/bin/python
import rospy
from dbw_mkz_msgs.msg import ThrottleCmd


rospy.init_node('TCommand', anonymous=True)
pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=10)
rate = rospy.Rate(10)
command = ThrottleCmd()
command.pedal_cmd = 1.0
command.pedal_cmd_type = 2
command.enable = 1
command.clear = 0
command.ignore = 0
command.count = 0
while not rospy.is_shutdown():
    pub.publish(command)
    rate.sleep()