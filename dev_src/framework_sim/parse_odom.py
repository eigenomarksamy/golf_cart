#! /usr/bin/env python


import os
import math
import numpy as np
import matplotlib.pyplot as plt


class FileOdomPoint:
    def __init__(self, seq, x, y, qx, qy, qz, qw):
        self._seq = seq
        self._x = x
        self._y = y
        self._qx = qx
        self._qy = qy
        self._qz = qz
        self._qw = qw


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return qx, qy, qz, qw


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


def list_to_obj(object):
    return object._seq, object._x, object._y, object._qx, object._qy, object._qz, object._qw


def get_course_path(obj_list):
    x_path = []
    y_path = []
    yaw_path = []
    for i in range(len(obj_list)):
        _, x, y, qx, qy, qz, qw = list_to_obj(obj_list[i])
        yaw, _, _ = quaternion_to_euler(qx, qy, qz, qw)
        x_path.append(x)
        y_path.append(y)
        yaw_path.append(yaw)
    return x_path, y_path, yaw_path


def plotter_xy(x_path, y_path):
    plt.figure()
    plt.plot(x_path, y_path, 'bo')
    plt.show()


def generate_txt(x_path, y_path, yaw_path):
    try:
        os.remove("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/x_path_txt.txt")
    except:
        pass
    try:
        os.remove("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/y_path_txt.txt")
    except:
        pass
    try:
        os.remove("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/yaw_path_txt.txt")
    except:
        pass
    x_file = open("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/x_path_txt.txt", "w")
    y_file = open("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/y_path_txt.txt", "w")
    yaw_file = open("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/yaw_path_txt.txt", "w")
    for i in range(len(x_path)):
        x_file.write(str(x_path[i]))
        x_file.write(" ")
    x_file.close()
    for i in range(len(y_path)):
        y_file.write(str(y_path[i]))
        y_file.write(" ")
    y_file.close()
    for i in range(len(yaw_path)):
        yaw_file.write(str(yaw_path[i]))
        yaw_file.write(" ")
    yaw_file.close()


def main():
    file_path = '/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/odom_data_file.txt'
    try:
        file = open(file_path)
    except:
        print "No File!"
    lines = file.readlines()
    odom_list = []
    for i in range(0, len(lines) - 7, 8):
        tmp_seq = float(lines[i])
        tmp_x = float(lines[i + 1])
        tmp_y = float(lines[i + 2])
        tmp_qx = float(lines[i + 3])
        tmp_qy = float(lines[i + 4])
        tmp_qz = float(lines[i + 5])
        tmp_qw = float(lines[i + 6])
        odom_obj = FileOdomPoint(tmp_seq, tmp_x, tmp_y, tmp_qx, tmp_qy, tmp_qz, tmp_qw)
        odom_list.append(odom_obj)
    file.close()
    x_path, y_path, yaw_path = get_course_path(odom_list)
    generate_txt(x_path, y_path, yaw_path)
    plotter_xy(x_path, y_path)


if __name__ == '__main__':
    main()