#! /usr/bin/python


import imp
import sys
import matplotlib.pyplot as plt
sys.path.append("/home/oks/catkin_ws/src/framework_sim/clrn_gplanner/scripts/")
import csp_generator


def get_path():
    waypoints_x = []
    waypoints_y = []
    target_velocity = 10.0 / 3.6
    # csp_generator = imp.load_source('csp_generator.py', '/home/oks/catkin_ws/src/framework_sim/clrn_gplanner/scripts/csp_generator.py')
    waypoints_x, waypoints_y, course_x, course_y, course_yaw, course_k, course_s, final_goal, speed_profile = csp_generator.generate_path(is_default = True)
    plan = [waypoints_x, waypoints_y, final_goal, course_x, course_y, course_yaw, course_k, course_s, target_velocity, speed_profile]
    csp_generator.generate_txt_course(plan)
    return plan


def plot_path(cx, cy):
    plt.plot(cx, cy, "-r", label="course")
    plt.show()


def main():
    plan = get_path()
    plot_path(plan[0], plan[1])


if __name__ == '__main__':
    main()