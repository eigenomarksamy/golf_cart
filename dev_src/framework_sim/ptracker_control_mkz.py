#! /usr/bin/python


import sys
import os
import time

import math
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("/home/oks/catkin_ws/src/framework_sim/clrn_ptracker/scripts/")
import controller
import configurator
sys.path.append("/home/oks/catkin_ws/src/framework_sim/clrn_analyzer/scripts/")
import live_plotter

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


def init_globals():
    global x_history_g, y_history_g, yaw_history_g, speed_history_g, time_history_g, seq_history_g, seq_pre_g, reached_the_end_g
    reached_the_end_g = False
    seq_pre_g = 0
    x_history_g = []
    y_history_g = []
    yaw_history_g = []
    speed_history_g = []
    time_history_g = []
    seq_history_g = []


def init_nav():
    init_globals()
    config = configurator.get_config()
    path = parse_path(config)
    pub_obj = init_ros(config)
    try:
        os.remove("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/controller_output.txt")
        os.remove("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/cont_fb_out.txt")
    except:
        pass
    return config, path, pub_obj


def init_ros(config):
    try:
        rospy.init_node(config.trkr_out_node)
        pub_obj = rospy.Publisher(config.trkr_out_topic, Float64MultiArray, queue_size=10)
        return pub_obj
    except rospy.ROSInterruptException:
        print('ROS Connection Lost!')
        return


def parse_path(config):
    course_txt = ["wpx\n", "wpy\n", "fg\n", "cx\n", "cy\n", "cyaw\n", "ck\n", "cs\n", "tv\n", "sp\n"]
    waypoints_x = []
    waypoints_y = []
    final_goal = []
    course_x = []
    course_y = []
    course_yaw = []
    course_k = []
    course_s = []
    target_velocity = []
    speed_profile = []
    try:
        course_file = open(config.gen_files_path, 'r')
        lines = course_file.readlines()
        for line in range(len(lines) - 1):
            if lines[line] == course_txt[0]:
                for i in lines[line + 1].split():
                    waypoints_x.append(float(i))
            elif lines[line] == course_txt[1]:
                for i in lines[line + 1].split():
                    waypoints_y.append(float(i))
            elif lines[line] == course_txt[2]:
                for i in lines[line + 1].split():
                    final_goal.append(float(i))
            elif lines[line] == course_txt[3]:
                for i in lines[line + 1].split():
                    course_x.append(float(i))
            elif lines[line] == course_txt[4]:
                for i in lines[line + 1].split():
                    course_y.append(float(i))
            elif lines[line] == course_txt[5]:
                for i in lines[line + 1].split():
                    course_yaw.append(float(i))
            elif lines[line] == course_txt[6]:
                for i in lines[line + 1].split():
                    course_k.append(float(i))
            elif lines[line] == course_txt[7]:
                for i in lines[line + 1].split():
                    course_s.append(float(i))
            elif lines[line] == course_txt[8]:
                for i in lines[line + 1].split():
                    target_velocity.append(float(i))
            elif lines[line] == course_txt[9]:
                for i in lines[line + 1].split():
                    speed_profile.append(float(i))
    except:
        print("No File!")
    return [waypoints_x, waypoints_y, final_goal, course_x, course_y, course_yaw, course_k, course_s, target_velocity, speed_profile]


def quaternion_to_euler(qx, qy, qz, qw):
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(t3, t4)
    return yaw, pitch, roll


def send_init_cmd(config, pub):
    cmd_list_msg = config.cmd_list_msg_t
    cmd_throttle = 0.0
    cmd_brake = 0.0
    cmd_steer = 0.0
    cmd_list_msg.data = [cmd_throttle, cmd_brake, cmd_steer]
    pub.publish(cmd_list_msg)


def convert_cmd_to_twist(cmd_throttle, cmd_steer, cmd_brake):
    global config_g
    twist_cmd = config_g.cmd_vel_msg_t
    if cmd_throttle > 0.0 and cmd_throttle <= 1.0:
        twist_cmd.linear.x = 1.0
    elif cmd_throttle == 0.0:
        twist_cmd.linear.x = 0.0
    twist_cmd.linear.x = cmd_throttle * 1.5
    if cmd_steer > 0.0:
        twist_cmd.angular.z = 1.0
    elif cmd_steer < 0.0:
        twist_cmd.angular.z = -1.0
    elif cmd_steer == 0.0:
        twist_cmd.angular.z = 0.0
    twist_cmd.linear.x = cmd_throttle
    twist_cmd.angular.z = cmd_steer / 1.22
    return twist_cmd


def convert_cmd_to_pub(cmd_throttle, cmd_steer, cmd_brake):
    global config_g
    cmd_list_msg = config_g.cmd_list_msg_t
    cmd_list_msg.data = [cmd_throttle, cmd_brake, cmd_steer]
    return cmd_list_msg


def logger(commands, feedback):
    commands = [str(commands[0]), str(commands[1]), str(commands[2])]
    feedback = [str(feedback[0]), str(feedback[1])]
    cmd_log = open("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/controller_output.txt", "a")
    fb_log = open("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/cont_fb_out.txt", "a")
    for i in range(len(commands)):
        cmd_log.write(commands[i])
        cmd_log.write("\n")
    cmd_log.write("-------------------------\n")
    cmd_log.close()
    for i in range(len(feedback)):
        fb_log.write(feedback[i])
        fb_log.write("\n")
    fb_log.write("-------------------------\n")
    fb_log.close()


def feedback_callback(odom_data):
    global cmd_pub_g, config_g, waypoints_g, x_history_g, y_history_g, yaw_history_g, speed_history_g, time_history_g, seq_history_g, seq_pre_g, controller_g, cur_time_g, pre_time_g, closest_index_g, fg_g, reached_the_end_g, wp_x_g, wp_y_g
    reached_the_end = reached_the_end_g
    seq_cur = odom_data.header.seq
    x_history = x_history_g
    y_history = y_history_g
    yaw_history = yaw_history_g
    time_history = time_history_g
    speed_history = speed_history_g
    seq_history = seq_history_g
    controller_l = controller_g
    waypoints = waypoints_g[0]
    waypoints_np = waypoints_g[1]
    wp_distance = waypoints_g[2]
    wp_interp = waypoints_g[3]
    wp_interp_hash = waypoints_g[4]
    odom_x = odom_data.pose.pose.position.x
    odom_y = odom_data.pose.pose.position.y
    odom_qx = odom_data.pose.pose.orientation.x
    odom_qy = odom_data.pose.pose.orientation.y
    odom_qz = odom_data.pose.pose.orientation.z
    odom_qw = odom_data.pose.pose.orientation.w
    odom_vx = odom_data.twist.twist.linear.x
    odom_vy = odom_data.twist.twist.linear.y
    odom_rz = odom_data.twist.twist.angular.z
    odom_yaw, _, _ = quaternion_to_euler(odom_qx, odom_qy, odom_qz, odom_qw)
    current_x = odom_x
    current_y = odom_y
    current_yaw = odom_yaw
    current_speed = np.sqrt(odom_vx**2 + odom_vy**2)
    x_history.append(current_x)
    y_history.append(current_y)
    yaw_history.append(current_yaw)
    speed_history.append(current_speed)
    seq_history.append(seq_cur)
    if seq_pre_g == 0:
        time_history.append(0.1)
        pre_time_g = 0.0
        cur_time_g = 0.1
    else:
        time_history.append(time_history[-1] + 0.1)
        cur_time_g = pre_time_g + 0.1
    previous_timestamp = pre_time_g
    current_timestamp = cur_time_g
    if seq_pre_g == 0:
        closest_index_g = 0
    closest_index = closest_index_g
    closest_distance = np.linalg.norm(np.array([
            waypoints_np[closest_index, 0] - current_x,
            waypoints_np[closest_index, 1] - current_y]))
    new_distance = closest_distance
    new_index = closest_index
    while new_distance <= closest_distance:
        closest_distance = new_distance
        closest_index = new_index
        new_index += 1
        if new_index >= waypoints_np.shape[0]:
            break
        new_distance = np.linalg.norm(np.array([
                waypoints_np[new_index, 0] - current_x,
                waypoints_np[new_index, 1] - current_y]))
    new_distance = closest_distance
    new_index = closest_index
    while new_distance <= closest_distance:
        closest_distance = new_distance
        closest_index = new_index
        new_index -= 1
        if new_index < 0:
            break
        new_distance = np.linalg.norm(np.array([
                waypoints_np[new_index, 0] - current_x,
                waypoints_np[new_index, 1] - current_y]))
    waypoint_subset_first_index = closest_index - 1
    if waypoint_subset_first_index < 0:
        waypoint_subset_first_index = 0
    waypoint_subset_last_index = closest_index
    total_distance_ahead = 0
    while total_distance_ahead < 20:
        total_distance_ahead += wp_distance[waypoint_subset_last_index]
        waypoint_subset_last_index += 1
        if waypoint_subset_last_index >= waypoints_np.shape[0]:
            waypoint_subset_last_index = waypoints_np.shape[0] - 1
            break
    new_waypoints = \
            wp_interp[wp_interp_hash[waypoint_subset_first_index]:\
                      wp_interp_hash[waypoint_subset_last_index] + 1]
    controller_l.update_waypoints(new_waypoints)
    controller_l.update_values(current_x, current_y, current_yaw, 
                             current_speed,
                             current_timestamp, 1)
    controller_l.update_controls()
    cmd_throttle, cmd_steer, cmd_brake = controller_l.get_commands()
    dist_to_last_waypoint = np.linalg.norm(np.array([
        waypoints[-1][0] - current_x,
        waypoints[-1][1] - current_y]))
    if  dist_to_last_waypoint < 10.0:
        reached_the_end = True
    if reached_the_end:
        print("Reaching the end of path.")
        cmd_throttle = 0.0
        cmd_brake = 1.0
    # msg_to_pub = convert_cmd_to_twist(cmd_throttle, cmd_steer, cmd_brake)
    msg_to_pub = convert_cmd_to_pub(cmd_throttle, cmd_steer, cmd_brake)
    cmd_pub_g.publish(msg_to_pub)
    commands = [cmd_throttle, cmd_steer, cmd_brake]
    feedback = [odom_x, odom_y]
    live_plotter.live_plot(wp_x_g, wp_y_g, odom_x, odom_y, closest_index)
    logger(commands, feedback)
    previous_timestamp = current_timestamp
    seq_pre_g = seq_cur
    # print "Seq INFO: ", seq_pre_g
    # print "Throttle CMD: ", cmd_throttle
    # print "Steer CMD: ", cmd_steer
    # print "Brake CMD: ", cmd_brake
    # print "X: ", current_x
    # print "Y: ", current_y
    # print "YAW: ", current_yaw
    # print "Speed: ", current_speed
    # print "Goal: ", fg_g[0], ", ", fg_g[1]
    # print "--------------------------"


def exec_nav(config, path, pub):
    global cmd_pub_g, waypoints_g, config_g, controller_g, fg_g, wp_x_g, wp_y_g
    cmd_pub_g = pub
    config_g = config
    wp_x = path[0]
    wp_y = path[1]
    fg = path[2]
    cx = path[3]
    cy = path[4]
    cyaw = path[5]
    ck = path[6]
    cs = path[7]
    tv = path[8]
    sp = path[9]
    v = []
    fg_g = fg
    for i in range(len(wp_x)):
        v.append(10.0)
    waypoints = list(zip(wp_x, wp_y, sp))
    waypoints_np = np.array(waypoints)
    wp_distance = []
    for i in range(1, waypoints_np.shape[0]):
        wp_distance.append(
                np.sqrt((waypoints_np[i, 0] - waypoints_np[i-1, 0])**2 +
                        (waypoints_np[i, 1] - waypoints_np[i-1, 1])**2))
    wp_distance.append(0)
    wp_interp      = []
    wp_interp_hash = []
    interp_counter = 0
    for i in range(waypoints_np.shape[0] - 1):
        wp_interp.append(list(waypoints_np[i]))
        wp_interp_hash.append(interp_counter)   
        interp_counter+=1
        num_pts_to_interp = int(np.floor(wp_distance[i] /\
                                     float(0.01)) - 1)
        wp_vector = waypoints_np[i+1] - waypoints_np[i]
        wp_uvector = wp_vector / np.linalg.norm(wp_vector)
        for j in range(num_pts_to_interp):
            next_wp_vector = 0.01 * float(j+1) * wp_uvector
            wp_interp.append(list(waypoints_np[i] + next_wp_vector))
            interp_counter+=1
    wp_interp.append(list(waypoints_np[-1]))
    wp_interp_hash.append(interp_counter)   
    interp_counter+=1
    num_iterations = 10
    if (10 < 1):
        num_iterations = 1
    waypoints_g = [waypoints, waypoints_np, wp_distance, wp_interp, wp_interp_hash]
    controller_l = controller.Controller(waypoints)
    controller_g = controller_l
    wp_x_g = wp_x
    wp_y_g = wp_y
    rospy.Subscriber(config.feedback_out_topic, Odometry, feedback_callback)
    send_init_cmd(config, pub)
    rospy.spin()


def main():
    while True:
        print('Get Ready for the Fuckin\' Ride')
        config, path, pub = init_nav()
        exec_nav(config, path, pub)
        print('Done.')
        return


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nYou chose to leave. Bye!')