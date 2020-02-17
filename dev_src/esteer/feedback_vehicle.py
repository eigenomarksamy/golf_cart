#! /usr/bin/env python


import os
import math
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from dbw_mkz_road.msg import Feedback


class FeedbackStates():

    def __init__(self, current_ts, current_x, current_y, current_vx, current_vy, current_yawr, current_quat):
        self.ts = current_ts
        self.x = current_x
        self.y = current_y
        self.vx = current_vx
        self.vy = current_vy
        self.yawr = current_yawr
        self.v = ((self.vx) ** 2 + (self.vy) ** 2) ** 0.5
        _, _, self.yaw = self.quaternion_to_euler(current_quat[0], current_quat[1], current_quat[2], current_quat[3])


    def quaternion_to_euler(self, qx, qy, qz, qw):
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
        return roll, pitch, yaw


def log_feedback(feedback_obj):
    odom_data_parse = [str(feedback_obj.ts), str(feedback_obj.x), str(feedback_obj.y), str(feedback_obj.yaw), str(feedback_obj.vx), str(feedback_obj.vy), str(feedback_obj.v), str(feedback_obj.yawr)]
    file = open('GenTxtDir/gen_feedback.txt', 'a')
    for i in range(len(odom_data_parse)):
        file.write(odom_data_parse[i])
        file.write("\n")
    file.write("-------------------------\n")
    file.close()



def fill_pub_feedback(feedback_obj, feedback_pub):
    feedback_msg = Feedback()
    feedback_msg.timestamp = feedback_obj.ts
    feedback_msg.global_x = feedback_obj.x
    feedback_msg.global_y = feedback_obj.y
    feedback_msg.global_yaw = feedback_obj.yaw
    feedback_msg.global_vx = feedback_obj.vx
    feedback_msg.global_vy = feedback_obj.vy
    feedback_msg.global_v = feedback_obj.v
    feedback_msg.global_yawr = feedback_obj.yawr
    feedback_pub.publish(feedback_msg)


def odom_callback(odom_data):
    global feedback_pub_g
    ts_cur = odom_data.header.stamp
    x_cur = odom_data.pose.pose.position.x
    y_cur = odom_data.pose.pose.position.y
    qx_cur = odom_data.pose.pose.orientation.x
    qy_cur = odom_data.pose.pose.orientation.y
    qz_cur = odom_data.pose.pose.orientation.z
    qw_cur = odom_data.pose.pose.orientation.w
    vx_cur = odom_data.twist.twist.linear.x
    vy_cur = odom_data.twist.twist.linear.y
    rz_cur = odom_data.twist.twist.angular.z
    quat_cur = [qx_cur, qy_cur, qz_cur, qw_cur]
    feedback_obj = FeedbackStates(ts_cur, x_cur, y_cur, vx_cur, vy_cur, rz_cur, quat_cur)
    feedback_pub = feedback_pub_g
    fill_pub_feedback(feedback_obj, feedback_pub)
    log_feedback(feedback_obj)


def main():
    global feedback_pub_g
    try:
        rospy.init_node('feedback_vehicle')
        feedback_pub = rospy.Publisher('/vehicle/feedback', Feedback, queue_size=10)
        feedback_pub_g = feedback_pub
        rospy.Subscriber('/vehicle/odom', Odometry, odom_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        print "ROS is not initialized!"

if __name__ == '__main__':
    dir_name = 'GenTxtDir'
    try:
        os.chdir('../')
        os.mkdir(dir_name)
    except OSError:
        pass
    try:
        os.remove('GenTxtDir/gen_feedback.txt')
    except:
        pass
    main()