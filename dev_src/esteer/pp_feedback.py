#! /usr/bin/env python


import os
import math
import numpy as np
import matplotlib.pyplot as plt


class FileFeedback:

    def __init__(self,ts, x, y, yaw, vx, vy, v, yawr):
        self._ts = ts
        self._x = x
        self._y = y
        self._yaw = yaw
        self._vx = vx
        self._vy = vy
        self._v = v
        self._yawr = yawr


def list_to_obj(object):
    return object._ts, object._x, object._y, object._yaw, object._vx, object._vy, object._v, object._yawr


def plotter(x_points, y_points, yaw_points, vx_points, vy_points, v_points, yawr_points,ts_points):

    for i in range(len(ts_points)):
        yaw_points[i] = math.degrees(np.float(yaw_points[i]))
        v_points[i] = v_points[i] * 3.6
               
    fig1, ((ax1, ax2, ax3,ax4)) = plt.subplots(4, 1)
    ax1.plot(x_points, y_points, 'b.-', label = 'X-Y', linewidth = 1)
    ax1.set_xlabel('global X-m')
    ax1.set_ylabel('global Y-m')
    ax1.grid()
    ax2.plot(ts_points, yaw_points, 'm.-', label = 'YAW', linewidth = 1)
    ax2.set_xlabel('Step')
    ax2.set_ylabel('global YAW Deg')
    ax2.grid()
    ax3.plot(ts_points, yaw_points, 'g.-', label = 'YAW-Rate', linewidth = 1)
    ax3.set_xlabel('Step')
    ax3.set_ylabel('global YAW Deg/s')
    ax3.grid()
    ax4.plot(ts_points, v_points, 'r.-', label = 'V', linewidth = 1)
    ax4.set_xlabel('Step')
    ax4.set_ylabel('global resultant V-km/hr')
    ax4.grid()
    plt.show()


def get_points(obj_list):
    ts_points = []
    x_points = []
    y_points = []
    yaw_points = []
    vx_points = []
    vy_points = []
    v_points = []
    yawr_points = []
    for i in range(len(obj_list)):
        ts, x, y, yaw, vx, vy, v, yawr = list_to_obj(obj_list[i])
        ts_points.append(ts)
        x_points.append(x)
        y_points.append(y)
        yaw_points.append(yaw)
        vx_points.append(vx)
        vy_points.append(vy)
        v_points.append(v)
        yawr_points.append(yawr)
    return ts_points, x_points, y_points, yaw_points, vx_points, vy_points, v_points, yawr_points


def main_feedback():
    try:
        os.chdir('../GenTxtDir/')
    except OSError:
        print "No ouput directory!"
        return
    try:
        gen_file = open('gen_feedback.txt')
        lines = gen_file.readlines()
        odom_list = []
        for i in range(0, len(lines) - 8, 9):
            tmp_ts = float(lines[i])
            tmp_x = float(lines[i + 1])
            tmp_y = float(lines[i + 2])
            tmp_yaw = float(lines[i + 3])
            tmp_vx = float(lines[i + 4])
            tmp_vy = float(lines[i + 5])
            tmp_v = float(lines[i + 6])
            tmp_yawr = float(lines[i + 7])
            odom_obj = FileFeedback(tmp_ts, tmp_x, tmp_y, tmp_yaw, tmp_vx, tmp_vy, tmp_v, tmp_yawr)
            odom_list.append(odom_obj)
        gen_file.close()
        ts_points, x_points, y_points, yaw_points, vx_points, vy_points, v_points, yawr_points = get_points(odom_list)
        plotter(x_points, y_points, yaw_points, vx_points, vy_points, v_points, yawr_points,ts_points)
    except OSError:
        print "No File!"
        return


def live_plot(cx, cy, x, y, target_ind):
    plt.cla()
    plt.plot(cx, cy, "-r", label="course")
    plt.plot(x, y, "ob", label="trajectory")
    plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
    plt.xlabel('X Global [m]')
    plt.ylabel('Y Global [m]')
    plt.legend()
    plt.grid(True)
    plt.ylim(-5,1)
    plt.title("Live Plotter")
    plt.pause(0.0001)


if __name__ == '__main__':
    main_feedback()