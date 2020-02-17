#! /usr/bin/python3


from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dbw_mkz_msgs.msg import ThrottleCmd, BrakeCmd, SteeringCmd, GearCmd, TurnSignalCmd
from std_msgs.msg import Float64


class Config():
    def __init__(self, vehicle = 'mkz'):
        self.gen_files_path = '/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/course_txt.txt'
        if vehicle == 'mkz':
            self.trkr_out_node = 'mkz_ptracker'
            self.trkr_out_topic =  '/mkz/ptracker'
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
        elif vehicle == 'blue':
            self.trkr_out_node = 'blue_ptracker'
            self.trkr_out_topic =  '/blue/ptracker'
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


def get_config(car = 'mkz'):
    if car == 'mkz':
        obj = Config()
    elif car == 'blue':
        obj = Config(car)
    return obj