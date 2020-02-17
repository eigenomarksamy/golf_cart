#! /usr/bin/env python


import rospy
from dbw_mkz_road.msg import ControlInput


def main():
    rospy.init_node('sample_control_pub')
    control_pub = rospy.Publisher('/vehicle/control_cmd', ControlInput, queue_size=10)
    r = rospy.Rate(10)
    control_msg = ControlInput()
    while not rospy.is_shutdown():
        control_msg.timestamp = rospy.Time.now()
        control_msg.cmd_velocity = 1.0  ## No upper limit
        control_msg.cmd_heading = 0.1   ## from 1.0 to -1.0, 0.0 is the center
        control_pub.publish(control_msg)
        r.sleep()


if __name__ == '__main__':
    main()