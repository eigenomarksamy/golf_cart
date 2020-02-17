#! /usr/bin/python


import rospy
import numpy as np
from dbw_mkz_road.msg import ControlInput
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd


def fill_msg(is_initialized, values = []):
    throttle_cmd = ThrottleCmd()
    brake_cmd = BrakeCmd()
    steering_cmd = SteeringCmd()
    if not is_initialized:
        throttle_cmd.pedal_cmd = 0.0
        throttle_cmd.pedal_cmd_type = 0
        throttle_cmd.enable = 0
        throttle_cmd.clear = 0
        throttle_cmd.ignore = 0
        brake_cmd.pedal_cmd = 0.0
        brake_cmd.pedal_cmd_type = 0
        brake_cmd.enable = 0
        brake_cmd.clear = 0
        brake_cmd.ignore = 0
        steering_cmd.steering_wheel_angle_cmd = 0.0
        steering_cmd.steering_wheel_angle_velocity = 0.0
        steering_cmd.steering_wheel_torque_cmd = 0.0
        steering_cmd.cmd_type = 0
        steering_cmd.enable = 0
        steering_cmd.clear = 0
        steering_cmd.ignore = 0
    else:
        throttle_cmd.pedal_cmd = values[0]
        throttle_cmd.pedal_cmd_type = values[1]
        throttle_cmd.enable = values[2]
        throttle_cmd.clear = values[3]
        throttle_cmd.ignore = values[4]
        brake_cmd.pedal_cmd = values[5]
        brake_cmd.pedal_cmd_type = values[6]
        brake_cmd.enable = values[7]
        brake_cmd.clear = values[8]
        brake_cmd.ignore = values[9]
        steering_cmd.steering_wheel_angle_cmd = values[10]
        steering_cmd.steering_wheel_angle_velocity = values[11]
        steering_cmd.steering_wheel_torque_cmd = values[12]
        steering_cmd.cmd_type = values[13]
        steering_cmd.enable = values[14]
        steering_cmd.clear = values[15]
        steering_cmd.ignore = values[16]
    return throttle_cmd, brake_cmd, steering_cmd


def lot_control(ts, velocity):
    global ts_pre, v_pre, v_err_pre, v_err_cur, v_err_pre_intg, v_err_intg, v_err_diff, throt_pre
    Kp = 1.0
    Ki = 0.1
    Kd = 0.01
    sample_t = 0.1
    if velocity < 0.0:
        velocity = 0.0
    v_des = velocity
    v_err_cur = v_des - v_pre
    v_err_intg = v_err_pre_intg + v_err_cur * sample_t
    v_err_diff = (v_err_cur - v_err_pre) / sample_t
    des_acc = Kp * (v_err_cur) + Ki * (v_err_intg) + Kd * (v_err_diff)
    if des_acc > 0:
        brake = 0.0
        throttle = (np.tanh(des_acc) + 1) / 2
        if (throttle - throt_pre) > 0.1:
            throttle = throt_pre + 0.1
    else:
        throttle = 0.0
        brake = 1.0
    return throttle, brake


def lat_control(heading):
    if heading > 1.0:
        heading = 1.0
    elif heading < -1.0:
        heading = -1.0
    heading *= 9.6
    steering = heading
    return steering


def controlcmd_callback(controlcmd_msg):
    global throttle_pub, brake_pub, steering_pub, ts_pre, v_pre, heading_pre, v_err_pre, v_err_cur, v_err_pre_intg, v_err_intg, throt_pre
    CMD_PERCENT = 2
    CMD_ANGLE = 0
    ts_cur = controlcmd_msg.timestamp
    des_velocity = controlcmd_msg.cmd_velocity
    des_heading = controlcmd_msg.cmd_heading
    throttle, brake = lot_control(ts_cur, des_velocity)
    steering = lat_control(des_heading)
    cmd_values = [throttle, CMD_PERCENT, True, True, True, brake, CMD_PERCENT, True, True, True, steering, 0.0, 0.0, CMD_ANGLE, True, True, True]
    throttle_msg, brake_msg, steering_msg = fill_msg(True, cmd_values)
    throttle_pub.publish(throttle_msg)
    brake_pub.publish(brake_msg)
    steering_pub.publish(steering_msg)
    heading_pre = des_heading
    v_pre = des_velocity
    v_err_pre = v_err_cur
    ts_pre = ts_cur
    v_err_pre_intg = v_err_intg
    throt_pre = throttle


def init_globals():
    global ts_pre, v_pre, v_err_pre, v_err_cur, v_err_pre_intg, v_err_intg, v_err_diff, throt_pre
    ts_pre = 0
    v_pre = 0.0
    heading_pre = 0.0
    v_err_pre = 0.0
    v_err_cur = 0.0
    v_err_pre_intg = 0.0
    v_err_intg = 0.0
    v_err_diff = 0.0
    throt_pre = 0.0


def main():
    global throttle_pub, brake_pub, steering_pub
    rospy.init_node('low_level_control', anonymous=True)
    throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=10)
    brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=10)
    steering_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=10)
    throttle_msg, brake_msg, steering_msg = fill_msg(False)
    init_globals()
    rospy.Subscriber('/vehicle/control_cmd', ControlInput, controlcmd_callback)
    throttle_pub.publish(throttle_msg)
    brake_pub.publish(brake_msg)
    steering_pub.publish(steering_msg)
    rospy.spin()


if __name__ == '__main__':
    main()