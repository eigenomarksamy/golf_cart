#! /usr/bin/env python


import os
import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry


def rec_callback(odom_data):
    global odom_header_seq, odom_header_stamp, odom_header_frameid, odom_childframeid, odom_pose_pose_position_x, odom_pose_pose_position_y, odom_pose_pose_orientation_x, odom_pose_pose_orientation_y, odom_pose_pose_orientation_z, odom_pose_pose_orientation_w
    odom_header_seq = odom_data.header.seq
    odom_pose_pose_position_x = odom_data.pose.pose.position.x
    odom_pose_pose_position_y = odom_data.pose.pose.position.y
    odom_pose_pose_orientation_x = odom_data.pose.pose.orientation.x
    odom_pose_pose_orientation_y = odom_data.pose.pose.orientation.y
    odom_pose_pose_orientation_z = odom_data.pose.pose.orientation.z
    odom_pose_pose_orientation_w = odom_data.pose.pose.orientation.w
    odom_data_parse = [str(odom_header_seq), str(odom_pose_pose_position_x), str(odom_pose_pose_position_y), str(odom_pose_pose_orientation_x), str(odom_pose_pose_orientation_y), str(odom_pose_pose_orientation_z), str(odom_pose_pose_orientation_w)]
    file = open("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/odom_data_file.txt", "a")
    for i in range(len(odom_data_parse)):
        file.write(odom_data_parse[i])
        file.write("\n")
    file.write("-------------------------\n")
    file.close()


def main():
    global node_name, subtopic_name
    rospy.init_node(node_name, anonymous = True)
    rospy.Subscriber(subtopic_name, Odometry, rec_callback)
    rospy.spin()


if __name__ == '__main__':
    node_name = 'rec_odom_mkz'
    subtopic_name = '/mkz/odom'
    odom_header_seq = []
    odom_header_stamp = []
    odom_header_frameid = []
    odom_childframeid = []
    odom_pose_pose_position_x = []
    odom_pose_pose_position_y = []
    odom_pose_pose_orientation_x = []
    odom_pose_pose_orientation_y = []
    odom_pose_pose_orientation_z = []
    odom_pose_pose_orientation_w = []
    try:
        os.remove("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/odom_data_file.txt")
    except:
        pass
    main()