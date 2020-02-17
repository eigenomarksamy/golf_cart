#! /usr/bin/env python


import os
import math
import numpy as np
import matplotlib.pyplot as plt


class FileFB:
    def __init__(self, x, y):
        self._x = x
        self._y = y


def list_to_obj(object):
    return object._x, object._y


def get_course_path(obj_list):
    x_path = []
    y_path = []
    for i in range(len(obj_list)):
        x, y= list_to_obj(obj_list[i])
        x_path.append(x)
        y_path.append(y)
    return x_path, y_path


def plotter(x_path, wp_x, y_path, wp_y):
    plt.plot(x_path, y_path, 'go')
    plt.plot(wp_x, wp_y, 'r-')
    plt.show()


def parse_wp_file():
    ref_file_path = '/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/course_txt.txt'
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
        course_file = open(ref_file_path, 'r')
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


def main():
    fb_file_path = '/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/cont_fb_out.txt'
    try:
        file = open(fb_file_path)
        lines = file.readlines()
        odom_list = []
        for i in range(0, len(lines) - 2, 3):
            tmp_x = float(lines[i])
            tmp_y = float(lines[i + 1])
            odom_obj = FileFB(tmp_x, tmp_y)
            odom_list.append(odom_obj)
        file.close()
        x_path, y_path = get_course_path(odom_list)
    except:
        print "No File!"
        return
    course_list = parse_wp_file()
    wp_x = course_list[0]
    wp_y = course_list[1]
    plotter(x_path, wp_x, y_path, wp_y)


if __name__ == '__main__':
    main()