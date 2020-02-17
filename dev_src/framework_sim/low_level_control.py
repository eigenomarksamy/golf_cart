#! /usr/bin/env python

import sys
import signal
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from dbw_mkz_msgs.msg import ThrottleCmd, BrakeCmd, SteeringCmd

sys.path.append("/home/oks/catkin_ws/src/framework_sim/clrn_cfg/scripts/")
import car_config_ns


class Cmd:
    def __init__(self, thrtl_obj, steer_obj, brake_obj, config):
        self.thrtl_pub = thrtl_obj
        self.steer_pub = steer_obj
        self.brake_pub = brake_obj
        self.thrtl_msg_t = config.thrtl_msg_t
        self.steer_msg_t = config.steer_msg_t
        self.brake_msg_t = config.brake_msg_t


def signal_handler(signal, frame):
    print("\nYou chose to leave. Goodbye!")
    sys.exit(0)


def init_ros(config):
    try:
        rospy.init_node(config.llc_out_node)
        if config.ns == 'fusion' or config.ns == 'mkz':
            pub_obj_thrtl = rospy.Publisher(config.thrtl_topic, ThrottleCmd, queue_size=10)
            pub_obj_steer = rospy.Publisher(config.steer_topic, SteeringCmd, queue_size=10)
            pub_obj_brake = rospy.Publisher(config.brake_topic, BrakeCmd, queue_size=10)
        elif config.ns == 'blue' or config.ns == 'orange':
            pub_obj_thrtl = rospy.Publisher(config.thrtl_topic, Float64, queue_size=10)
            pub_obj_steer = rospy.Publisher(config.steer_topic, Float64, queue_size=10)
            pub_obj_brake = rospy.Publisher(config.brake_topic, Float64, queue_size=10)
        cmd_obj = Cmd(pub_obj_thrtl, pub_obj_steer, pub_obj_brake, config)
        return cmd_obj
    except rospy.ROSInterruptException:
        print('ROS Connection Lost!')
        return


def fill_thrtl_msg(msg, values):
    thrtl_msg = msg
    thrtl_msg.pedal_cmd = values[0]
    thrtl_msg.pedal_cmd_type = values[1]
    thrtl_msg.enable = values[2]
    thrtl_msg.clear = values[3]
    thrtl_msg.ignore = values[4]
    return thrtl_msg


def fill_brake_msg(msg, values):
    brake_msg = msg
    brake_msg.pedal_cmd = values[0]
    brake_msg.pedal_cmd_type = values[1]
    brake_msg.enable = values[2]
    brake_msg.clear = values[3]
    brake_msg.ignore = values[4]
    return brake_msg


def fill_steer_msg(msg, values):
    steer_msg = msg
    steer_msg.steering_wheel_angle_cmd = values[0] * 9.6
    steer_msg.steering_wheel_angle_velocity = values[1]
    steer_msg.steering_wheel_torque_cmd = values[2]
    steer_msg.cmd_type = values[3]
    steer_msg.enable = values[4]
    steer_msg.clear = values[5]
    steer_msg.ignore = values[6]
    return steer_msg


def publish_steer(steer_pub, steer_msg, steer_cmd_list, is_direct):
    print "Publishing Steering"
    steer_pub_obj = steer_pub
    if not is_direct:
        steer_msg = fill_steer_msg(steer_msg, steer_cmd_list)
    else:
        steer_msg.data = steer_cmd_list * 17.3
    steer_pub_obj.publish(steer_msg)
    print "Published Steering"


def publish_brake(brake_pub, brake_msg, brake_cmd_list, is_direct):
    print "Publishing Brake"
    brake_pub_obj = brake_pub
    if not is_direct:
        brake_msg = fill_brake_msg(brake_msg, brake_cmd_list)
    else:
        brake_msg.data = brake_cmd_list
    brake_pub_obj.publish(brake_msg)
    print "Published Brake"


def publish_thrtl(thrtl_pub, thrtl_msg, thrtl_cmd_list, is_direct):
    print "Publishing Throttle"
    thrtl_pub = thrtl_pub
    if not is_direct:
        thrtl_msg = fill_thrtl_msg(thrtl_msg, thrtl_cmd_list)
    else:
        thrtl_msg.data = thrtl_cmd_list
    thrtl_pub.publish(thrtl_msg)
    print "Published Throttle"


def publish_cmd(cmd_obj, conf_obj, cmd_list):
    thrtl_pub = cmd_obj.thrtl_pub
    steer_pub = cmd_obj.steer_pub
    brake_pub = cmd_obj.brake_pub
    thrtl_msg = cmd_obj.thrtl_msg_t
    brake_msg = cmd_obj.brake_msg_t
    steer_msg = cmd_obj.steer_msg_t
    if conf_obj.ns == 'fusion' or conf_obj.ns == 'mkz':
        is_direct = False
        thrtl_cmd_list = [0.0] * 5
        brake_cmd_list = [0.0] * 5
        steer_cmd_list = [0.0] * 7
        thrtl_start_index = 0
        brake_start_index = len(thrtl_cmd_list)
        steer_start_index = len(thrtl_cmd_list) + len(brake_cmd_list)
        thrtl_cmd_list = cmd_list[0:brake_start_index]
        for i in range(len(brake_cmd_list)):
            brake_cmd_list[i] = cmd_list[i + brake_start_index]
        steer_cmd_list = cmd_list[steer_start_index:]
        thrtl_cmd = thrtl_cmd_list
        brake_cmd = brake_cmd_list
        steer_cmd = steer_cmd_list
    elif conf_obj.ns == 'blue' or conf_obj.ns == 'orange':
        is_direct = True
        thrtl_cmd = cmd_list[0]
        brake_cmd = cmd_list[1]
        steer_cmd = cmd_list[2]
    print is_direct
    print thrtl_cmd
    print brake_cmd
    print steer_cmd
    publish_thrtl(thrtl_pub, thrtl_msg, thrtl_cmd, is_direct)
    publish_brake(brake_pub, brake_msg, brake_cmd, is_direct)
    publish_steer(steer_pub, steer_msg, steer_cmd, is_direct)


def cmd_callback(msg):
    global cmd_obj_g, config_obj_g
    cmd_obj = cmd_obj_g
    config_obj = config_obj_g
    cmd_list_in = msg.data
    if config_obj.ns == 'fusion' or config_obj.ns == 'mkz':
        CMD_NONE = 0
        CMD_PEDAL = 1
        CMD_PERCENT = 2
        CMD_TORQUE = 3
        CMD_TORQUE_RQ = 4
        CMD_DECEL = 6
        TORQUE_BOO = 520.0
        TORQUE_MAX = 3412.0
        CMD_ANGLE = 0
        CMD_TORQUE = 1
        ANGLE_MAX = 9.6
        VELOCITY_MAX = 17.5
        TORQUE_MAX = 8.0
        cmd_list = [0] * 17
        cmd_list[0] = cmd_list_in[0]
        cmd_list[1] = CMD_PERCENT
        cmd_list[2] = cmd_list[3] = cmd_list[4] = True
        cmd_list[5] = cmd_list_in[1]
        cmd_list[6] = cmd_list[1]
        cmd_list[7] = cmd_list[8] = cmd_list[9] = cmd_list[2]
        cmd_list[10] = cmd_list_in[2]
        cmd_list[11] = cmd_list[12] = 0.0
        cmd_list[13] = CMD_ANGLE
        cmd_list[14] = cmd_list[15] = cmd_list[16] = cmd_list[7]
    elif config_obj.ns == 'blue' or config_obj.ns == 'orange':
        cmd_list = [0] * len(cmd_list_in)
        for i in range(len(cmd_list_in)):
            cmd_list[i] = cmd_list_in[i]
    print cmd_list
    publish_cmd(cmd_obj, config_obj, cmd_list)


def exec_ros(cmd_obj, config_obj):
    global cmd_obj_g, config_obj_g
    if config_obj.ns == 'fusion' or config_obj.ns == 'mkz':
        CMD_NONE = 0
        CMD_PEDAL = 1
        CMD_PERCENT = 2
        CMD_TORQUE = 3
        CMD_TORQUE_RQ = 4
        CMD_DECEL = 6
        TORQUE_BOO = 520.0
        TORQUE_MAX = 3412.0
        CMD_ANGLE = 0
        CMD_TORQUE = 1
        ANGLE_MAX = 9.6
        VELOCITY_MAX = 17.5
        TORQUE_MAX = 8.0
        init_cmd_list = [0.0, CMD_NONE, True, True, True, 1.0, CMD_NONE, True, True, True, 0.0, 0.0, 0.0, CMD_ANGLE, True, True, True]
    elif config_obj.ns == 'blue' or config_obj.ns == 'orange':
        init_cmd_list = [0.0, 0.0, 0.0]
    cmd_obj_g = cmd_obj
    config_obj_g = config_obj
    rospy.Subscriber(config_obj.trkr_out_topic, Float64MultiArray, cmd_callback)
    publish_cmd(cmd_obj, config_obj, init_cmd_list)
    rospy.spin()


def main():
    v_ns = 'blue'
    config_obj = car_config_ns.CarConfigNS(v_ns)
    cmd_obj = init_ros(config_obj)
    exec_ros(cmd_obj, config_obj)


if __name__ == '__main__':
    main()