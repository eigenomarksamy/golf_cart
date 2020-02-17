#! /usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dbw_mkz_msgs.msg import ThrottleCmd, BrakeCmd, SteeringCmd, GearCmd, TurnSignalCmd


class Vehicle_NS:
    def __init__(self, vehicle):
        self.set_vehicle_namespace(vehicle)

    def set_vehicle_namespace(self, vehicle):
        if vehicle == "mkz":
            self.trkr_out_llc_in_node = 'mkz_llc_node'
            self.trkr_out_llc_in_topic =  '/mkz/ptracker'
            self.feedback_out_llc_in_topic = '/mkz/odom'
            self.thrtl_topic = '/mkz/throttle_cmd'
            self.brake_topic = '/mkz/brake_cmd'
            self.steer_topic = '/mkz/steering_cmd'
            self.gear_topic = '/mkz/gear_cmd'
            self.cmd_vel_topic = 'mkz/cmd_vel'
            self.turn_signal_topic = '/mkz/turn_signal_cmd'
            self.trkr_msg_t = Float64MultiArray()
            self.feedback_msg_t = Odometry()
            self.thrtl_msg_t = ThrottleCmd()
            self.brake_msg_t = BrakeCmd()
            self.steer_msg_t = SteeringCmd()
            self.gear_msg_t = GearCmd()
            self.turn_signal_msg_t = TurnSignalCmd()
            self.cmd_vel_msg_t = Twist()


        elif vehicle == "fusion":
            self.trkr_out_llc_in_node = 'fusion_llc_node'
            self.trkr_out_llc_in_topic =  '/fusion/ptracker'
            self.thrtl_topic = '/fusion/throttle_cmd'
            self.brake_topic = '/fusion/brake_cmd'
            self.steer_topic = '/fusion/steering_cmd'
            self.gear_topic = '/fusion/gear_cmd'
            self.turn_signal_topic = '/fusion/turn_signal_cmd'
            self.cmd_vel_topic = 'fusion/cmd_vel'
            self.thrtl_msg_t = ThrottleCmd()
            self.brake_msg_t = BrakeCmd()
            self.steer_msg_t = SteeringCmd()
            self.gear_msg_t = GearCmd()
            self.turn_signal_msg_t = TurnSignalCmd()
            self.cmd_vel_msg_t = Twist()


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


def get_cur_states(fb_obj):
    cur_x = fb_obj.pose.pose.position.x
    cur_y = fb_obj.pose.pose.position.y
    cur_qx = fb_obj.pose.pose.orientation.x
    cur_qy = fb_obj.pose.pose.orientation.y
    cur_qz = fb_obj.pose.pose.orientation.z
    cur_qw = fb_obj.pose.pose.orientation.w
    cur_yaw, _, _ = quaternion_to_euler(cur_qx, cur_qy, cur_qz, cur_qw)
    return cur_x, cur_y, cur_yaw


def acquire_control(ref_pos, cur_states):
    ref_x = ref_pos[0]
    ref_y = ref_pos[1]
    cur_x = cur_states[0]
    cur_y = cur_states[1]
    ref_yaw = math.atan2(ref_pos[1], ref_pos[0])
    if ref_yaw > cur_states[2]:
        direction = 1.0
    elif ref_yaw < cur_states[2]:
        direction = -1.0
    else:
        direction = 0.0
    control_angr = direction * 1.0
    reference = ((ref_x - cur_x) ** 2 + (ref_y - cur_y) ** 2) ** 0.5
    print "ref: ", ref_pos
    control_linv = reference
    return [control_linv, control_angr]


def llc(fb_obj):
    global reference_x, reference_y, ns_obj
    cur_x, cur_y, cur_yaw = get_cur_states(fb_obj)
    print "Cur States: ", cur_x, cur_y, cur_yaw
    cur_states = [cur_x, cur_y, cur_yaw]
    ref_pos = [reference_x, reference_y]
    control_action = acquire_control(ref_pos, cur_states)
    ret_val = ns_obj.cmd_vel_msg_t
    ret_val.linear.x = control_action[0]
    ret_val.linear.y = 0.0
    ret_val.linear.z = 0.0
    ret_val.angular.x = 0.0
    ret_val.angular.y = 0.0
    ret_val.angular.z = control_action[1]
    return ret_val


def feedback_out_llc_in_callback(odom_feedback_msg):
    global cmd_vel_pub
    while odom_feedback_msg.header.seq > 4:
        control_input = llc(odom_feedback_msg)
        cmd_vel_pub.publish(control_input)
        print "control_input: ", control_input


def main():
    global cmd_vel_pub, ns_obj, reference_x, reference_y
    try:
        reference_x = 10.0
        reference_y = -10.0
        ns_obj = Vehicle_NS("mkz")
        rospy.init_node(ns_obj.trkr_out_llc_in_node, anonymous=True)
        # thrtl_pub = rospy.Publisher(ns_obj.thrtl_topic, ns_obj.thrtl_msg_t, queue_size=10)
        # brake_pub = rospy.Publisher(ns_obj.brake_topic, ns_obj.brake_msg_t, queue_size=10)
        # steer_pub = rospy.Publisher(ns_obj.steer_topic, ns_obj.steer_msg_t, queue_size=10)
        # gear_pub = rospy.Publisher(ns_obj.gear_topic, ns_obj.gear_msg_t, queue_size=10)
        cmd_vel_pub = rospy.Publisher(ns_obj.cmd_vel_topic, Twist, queue_size=10)
        cmd_vel_init_msg = ns_obj.cmd_vel_msg_t
        cmd_vel_init_msg.linear.x = 0.0
        cmd_vel_init_msg.linear.y = 0.0
        cmd_vel_init_msg.linear.z = 0.0
        cmd_vel_init_msg.angular.x = 0.0
        cmd_vel_init_msg.angular.y = 0.0
        cmd_vel_init_msg.angular.z = 0.0
        # rospy.Subscriber(ns_obj.trkr_out_llc_in_topic, ns_obj.trkr_msg_t, trkr_out_llc_in_callback)
        rospy.Subscriber(ns_obj.feedback_out_llc_in_topic, Odometry, feedback_out_llc_in_callback)
        cmd_vel_pub.publish(cmd_vel_init_msg)
        rospy.spin()
    except any as exception:
        print exception


if __name__ == '__main__':
    main()