#! /usr/bin/env python


import rospy
from dbw_mkz_road.msg import Feedback


def feedback_callback(feedback_msg):
    cur_ts = feedback_msg.timestamp
    cur_x = feedback_msg.global_x
    cur_y = feedback_msg.global_y
    cur_yaw = feedback_msg.global_yaw
    cur_vx = feedback_msg.global_vx
    cur_vy = feedback_msg.global_vy
    cur_v = feedback_msg.global_v
    cur_yawr = feedback_msg.global_yawr
    print "TimeStamp: ", cur_ts
    print "Current X: ", cur_x
    print "Current Y: ", cur_y
    print "Current YAW: ", cur_yaw
    print "Current VX: ", cur_vx
    print "Current VY: ", cur_vy
    print "Current V: ", cur_v
    print "Current YAWR: ", cur_yawr


def main():
    rospy.init_node('sample_feedback_listener')
    rospy.Subscriber('/vehicle/feedback', Feedback, feedback_callback)
    rospy.spin()


if __name__ == '__main__':
    main()