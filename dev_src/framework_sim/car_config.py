#! /usr/bin/env python


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray
from dbw_mkz_msgs.msg import TwistCmd, GearCmd, ThrottleCmd, BrakeCmd, SteeringCmd, TurnSignalCmd


class CarConfig:
    def __init__(self, vehicle_ns = 'fusion'):
        self.ns = vehicle_ns
        if vehicle_ns == 'fusion':
            self.llc_out_node = 'fusion_llc'
            self.llc_out_topic =  '/fusion/llc'
            self.trkr_out_topic = '/fusion/ptracker'
            self.feedback_out_topic = '/fusion/odom'
            self.thrtl_topic = '/fusion/throttle_cmd'
            self.brake_topic = '/fusion/brake_cmd'
            self.steer_topic = '/fusion/steering_cmd'
            self.gear_topic = '/fusion/gear_cmd'
            self.cmd_vel_topic = '/fusion/cmd_vel'
            self.turn_signal_topic = '/fusion/turn_signal_cmd'
            self.feedback_msg_t = Odometry()
            self.thrtl_msg_t = ThrottleCmd()
            self.brake_msg_t = BrakeCmd()
            self.steer_msg_t = SteeringCmd()
            self.gear_msg_t = GearCmd()
            self.turn_signal_msg_t = TurnSignalCmd()
            self.cmd_vel_msg_t = Twist()
            self.cmd_list_msg_t = Float64MultiArray()
            self.time_step = 0.1
        elif vehicle_ns == 'mkz':
            self.llc_out_node = 'mkz_llc'
            self.llc_out_topic =  '/mkz/llc'
            self.trkr_out_topic = '/mkz/ptracker'
            self.feedback_out_topic = '/mkz/odom'
            self.thrtl_topic = '/mkz/throttle_cmd'
            self.brake_topic = '/mkz/brake_cmd'
            self.steer_topic = '/mkz/steering_cmd'
            self.gear_topic = '/mkz/gear_cmd'
            self.cmd_vel_topic = '/mkz/cmd_vel'
            self.turn_signal_topic = '/mkz/turn_signal_cmd'
            self.feedback_msg_t = Odometry()
            self.thrtl_msg_t = ThrottleCmd()
            self.brake_msg_t = BrakeCmd()
            self.steer_msg_t = SteeringCmd()
            self.gear_msg_t = GearCmd()
            self.turn_signal_msg_t = TurnSignalCmd()
            self.cmd_vel_msg_t = Twist()
            self.cmd_list_msg_t = Float64MultiArray()
            self.time_step = 0.1
        elif vehicle_ns == 'blue':
            self.llc_out_node = 'blue_llc'
            self.llc_out_topic =  '/blue/llc'
            self.trkr_out_topic = '/blue/ptracker'
            self.feedback_out_topic = '/blue/odom'
            self.thrtl_topic = '/blue/throttle_cmd'
            self.brake_topic = '/blue/brake_cmd'
            self.steer_topic = '/blue/steering_cmd'
            self.cmd_vel_topic = '/blue/cmd_vel'
            self.feedback_msg_t = Odometry()
            self.thrtl_msg_t = Float64()
            self.brake_msg_t = Float64()
            self.steer_msg_t = Float64()
            self.cmd_vel_msg_t = Twist()
            self.cmd_list_msg_t = Float64MultiArray()
            self.time_step = 0.1
        elif vehicle_ns == 'orange':
            self.llc_out_node = 'orange_llc'
            self.llc_out_topic =  '/orange/llc'
            self.trkr_out_topic = '/orange/ptracker'
            self.feedback_out_topic = '/orange/odom'
            self.thrtl_topic = '/orange/throttle_cmd'
            self.brake_topic = '/orange/brake_cmd'
            self.steer_topic = '/orange/steering_cmd'
            self.cmd_vel_topic = '/orange/cmd_vel'
            self.feedback_msg_t = Odometry()
            self.thrtl_msg_t = Float64()
            self.brake_msg_t = Float64()
            self.steer_msg_t = Float64()
            self.cmd_vel_msg_t = Twist()
            self.cmd_list_msg_t = Float64MultiArray()
            self.time_step = 0.1