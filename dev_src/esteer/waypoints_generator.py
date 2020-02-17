#! /usr/bin/env python


import os
import sys
import numpy as np
import matplotlib.pyplot as plt

########################################################################################
########################################################################################
########    This script takes an optional argument                              ########
########    which matches to one of the following:                              ########
########        > **No Argument**   --> Default straight line path              ########
########        > default           --> Default straight line path              ########
########        > defbez            --> Default bezier path                     ########
########        > dlane             --> Double lane maneuver                    ########
########        > b2b               --> Double lane maneuver, returning back    ########
########        > wlane             --> Wide lane maneuver                      ########
########    And an optional second argument regarding the velocity profile      ########
########    Note: the argument must be the 2nd one. And it's something of       ########
########    the following:                                                      ########
########        > **No Argument**   --> Default velocity profile                ########
########        > trap1             --> Trapezuidal of linear ramp              ########
########        > trap2             --> Trapezuidal of polynomial ramp          ########
########        > **Anything else** --> Default velocity profile                ########
########################################################################################
########################################################################################

class PathProps:
    def __init__(self, is_default = True, mode = 'default'):
        self._is_default = is_default
        self._mode = mode
        if is_default:
            self._x_initial = 0.0
            self._y_initial = -4.335
            self._x_final = 600.0
            self._y_final = self._y_initial
            self._target_speed = 10.0
            self._step = 1.0
            self._path_len = int((self._x_final - self._x_initial) / self._step)
            self._max_accel_decel = 1.0
        else:
            if self._mode == 'defbez':
                self.set_path_params(600.0, 0.0, 0.0, -4.335, 10.0)
            elif self._mode == 'dlane':
                self.set_path_params(600.0, 4.335, 0.0, -4.335, 10.0)
            elif self._mode == 'b2b':
                self.set_path_params(600.0, -4.335, 0.0, -4.335, 10.0)
            elif self._mode == 'wlane':
                self.set_path_params(600.0, 4.335, 0.0, -4.335, 10.0)

    def set_path_params(self, x_final, y_final, x_initial, y_initial, ts):
        self._x_final = x_final
        self._x_initial = x_initial
        self._y_final = y_final
        self._y_initial = y_initial
        self._target_speed = ts
        self._max_accel_decel = 1.0
        self._yf_list = [self._y_initial]
        self._step = 1.0
        self._path_len = int((self._x_final - self._x_initial) / self._step)
        if self._mode == 'defbez':
            self._man_width = 30.0
            self._y_const = 1
            self._x_chl_list = [100]
            for i in range(self._y_const):
                self._yf_list.append(self._yf_list[-1] + 4.335)
        elif self._mode == 'dlane':
            self._man_width = 30.0
            self._y_const = 2
            self._x_chl_list = [100, 170]
            for i in range(self._y_const):
                self._yf_list.append(self._yf_list[-1] + 4.335)
        elif self._mode == 'b2b':
            self._man_width = 30.0
            self._y_const = 2
            self._x_chl_list = [100, 400]
            for i in range(self._y_const):
                self._yf_list.append(self._yf_list[-1] + ((-1)**i) * 4.335)
        elif self._mode == 'wlane':
            self._man_width = 60.0
            self._y_const = 1
            self._x_chl_list = [100]
            for i in range(self._y_const):
                self._yf_list.append(self._yf_list[-1] + 2 * 4.335)

    def generate_path(self):
        if self._is_default:
            self._x_path_arr = np.arange(self._x_initial, (self._x_final + self._step), self._step)
            self._y_path_arr = np.array([self._y_initial])
            self._yaw_path_arr = np.array([0.0])
            for i in range(self._path_len):
                self._y_path_arr = np.append(self._y_path_arr, [self._y_initial])
                self._yaw_path_arr = np.append(self._yaw_path_arr, [0.0])
        else:
            if self._mode == 'defbez':
                x_start_man = self._x_chl_list[self._y_const - 1]
                y_start_man = self._y_initial
                y_end_man = self._yf_list[self._y_const]
                self._x_path_arr = np.arange(self._x_initial, x_start_man, self._step)
                self._y_path_arr = np.array([self._y_initial])
                self._yaw_path_arr = np.array([0.0])
                for i in range(len(self._x_path_arr) - 1):
                    self._y_path_arr = np.append(self._y_path_arr, [self._yf_list[self._y_const - 1]])
                    self._yaw_path_arr = np.append(self._yaw_path_arr, [0.0])
                self.generate_bez_man(x_start_man, y_start_man, y_end_man)
                for i in range(len(self._x_path_arr), self._path_len):
                    self._x_path_arr = np.append(self._x_path_arr, [self._x_path_arr[-1] + self._step])
                    self._y_path_arr = np.append(self._y_path_arr, [y_end_man])
                    self._yaw_path_arr = np.append(self._yaw_path_arr, [0.0])
            elif self._mode == 'dlane':
                x_start_man = self._x_chl_list[self._y_const - 2]
                y_start_man = self._y_initial
                y_end_man = self._yf_list[self._y_const - 1]
                self._x_path_arr = np.arange(self._x_initial, x_start_man, self._step)
                self._y_path_arr = np.array([self._y_initial])
                self._yaw_path_arr = np.array([0.0])
                for i in range(len(self._x_path_arr) - 1):
                    self._y_path_arr = np.append(self._y_path_arr, [self._yf_list[self._y_const - 2]])
                    self._yaw_path_arr = np.append(self._yaw_path_arr, [0.0])
                self.generate_bez_man(x_start_man, y_start_man, y_end_man)
                x_start_man = self._x_chl_list[self._y_const - 1]
                y_start_man = self._yf_list[self._y_const - 1]
                y_end_man = self._yf_list[self._y_const]
                for i in range(len(self._x_path_arr), x_start_man):
                    self._x_path_arr = np.append(self._x_path_arr, [self._x_path_arr[-1] + self._step])
                    self._y_path_arr = np.append(self._y_path_arr, [self._yf_list[self._y_const - 1]])
                    self._yaw_path_arr = np.append(self._yaw_path_arr, [0.0])
                self.generate_bez_man(x_start_man, y_start_man, y_end_man)
                for i in range(len(self._x_path_arr), self._path_len):
                    self._x_path_arr = np.append(self._x_path_arr, [self._x_path_arr[-1] + self._step])
                    self._y_path_arr = np.append(self._y_path_arr, [y_end_man])
                    self._yaw_path_arr = np.append(self._yaw_path_arr, [0.0])
            elif self._mode == 'b2b':
                x_start_man = self._x_chl_list[self._y_const - 2]
                y_start_man = self._y_initial
                y_end_man = self._yf_list[self._y_const - 1]
                self._x_path_arr = np.arange(self._x_initial, x_start_man, self._step)
                self._y_path_arr = np.array([self._y_initial])
                self._yaw_path_arr = np.array([0.0])
                for i in range(len(self._x_path_arr) - 1):
                    self._y_path_arr = np.append(self._y_path_arr, [self._yf_list[self._y_const - 2]])
                    self._yaw_path_arr = np.append(self._yaw_path_arr, [0.0])
                self.generate_bez_man(x_start_man, y_start_man, y_end_man)
                x_start_man = self._x_chl_list[self._y_const - 1]
                y_start_man = self._yf_list[self._y_const - 1]
                y_end_man = self._yf_list[self._y_const]
                for i in range(len(self._x_path_arr), x_start_man):
                    self._x_path_arr = np.append(self._x_path_arr, [self._x_path_arr[-1] + self._step])
                    self._y_path_arr = np.append(self._y_path_arr, [self._yf_list[self._y_const - 1]])
                    self._yaw_path_arr = np.append(self._yaw_path_arr, [0.0])
                self.generate_bez_man(x_start_man, y_start_man, y_end_man)
                for i in range(len(self._x_path_arr), self._path_len):
                    self._x_path_arr = np.append(self._x_path_arr, [self._x_path_arr[-1] + self._step])
                    self._y_path_arr = np.append(self._y_path_arr, [y_end_man])
                    self._yaw_path_arr = np.append(self._yaw_path_arr, [0.0])
            elif self._mode == 'wlane':
                x_start_man = self._x_chl_list[self._y_const - 1]
                y_start_man = self._y_initial
                y_end_man = self._yf_list[self._y_const]
                self._x_path_arr = np.arange(self._x_initial, x_start_man, self._step)
                self._y_path_arr = np.array([self._y_initial])
                self._yaw_path_arr = np.array([0.0])
                for i in range(len(self._x_path_arr) - 1):
                    self._y_path_arr = np.append(self._y_path_arr, [self._yf_list[self._y_const - 1]])
                    self._yaw_path_arr = np.append(self._yaw_path_arr, [0.0])
                self.generate_bez_man(x_start_man, y_start_man, y_end_man)
                for i in range(len(self._x_path_arr), self._path_len):
                    self._x_path_arr = np.append(self._x_path_arr, [self._x_path_arr[-1] + self._step])
                    self._y_path_arr = np.append(self._y_path_arr, [y_end_man])
                    self._yaw_path_arr = np.append(self._yaw_path_arr, [0.0])
        return self._x_path_arr, self._y_path_arr
    
    def generate_bez_man(self, x_start, y_start, y_end):
        x_end = x_start + self._man_width
        bez_iterations = int(x_end - x_start)
        control_p1 = [x_start, y_start]
        control_p2 = [x_end - 10.0, y_start]
        control_p3 = [x_start + 10.0, y_end]
        control_p4 = [x_end, y_end]
        cp = [control_p1, control_p2, control_p3, control_p4]
        [x_bez, y_bez] = self.Bezier(cp, bez_iterations)
        self.append_to_path([x_bez, y_bez], x_start, x_end)

    def append_to_path(self, xy_pts, x_start, x_end):
        it_start = len(self._x_path_arr)
        it_end = int(it_start + self._man_width)
        for i in range(it_start, it_end):
            bez_i = i - it_start
            self._x_path_arr = np.append(self._x_path_arr, [xy_pts[0][bez_i]])
            self._y_path_arr = np.append(self._y_path_arr, [xy_pts[1][bez_i]])
            self._yaw_path_arr = np.append(self._yaw_path_arr, [np.arctan2([xy_pts[0][bez_i]], [xy_pts[1][bez_i]])])
    
    def Bezier(self, cp, points):
        bx =  np.zeros(points, dtype=np.double)
        by =  np.zeros(points, dtype=np.double)
        p1_x = cp[0][0]
        p1_y = cp[0][1]
        p2_x = cp[1][0]
        p2_y = cp[1][1]
        p3_x = cp[2][0]
        p3_y = cp[2][1]
        p4_x = cp[3][0]
        p4_y = cp[3][1]
        for t1 in range(points):
            t = t1 * 1.0/points
            bx[t1] = (((1-t)**3) * p1_x) + ( 3*((1-t)**2) * t * p2_x) + ( 3*(1-t) * t**2 * p3_x) + (t**3 * p4_x)
            by[t1] = (((1-t)**3) * p1_y) + ( 3*((1-t)**2) * t * p2_y) + ( 3*(1-t) * t**2 * p3_y) + (t**3 * p4_y)
        return bx, by

    def generate_nvp(self):
        self._x_path_list = list(self._x_path_arr)
        self._yaw_path_list = list(self._yaw_path_arr)
        self._speed_profile = [self._target_speed] * len(self._x_path_list)
        self.tmp_direction = 1.0
        for i in range(len(self._x_path_list) - 1):
            self.d_yaw = abs(self._yaw_path_list[i + 1] - self._yaw_path_list[i])
            switch = np.pi / 4.0 <= self.d_yaw < np.pi / 2.0
            if switch:
                self.tmp_direction *= -1
            if self.tmp_direction != 1.0:
                self._speed_profile[i] = - self._target_speed
            else:
                self._speed_profile[i] = self._target_speed
            if switch:
                self._speed_profile[i] = 0.0
        for i in range(40):
            self._speed_profile[-i] = self._target_speed / (50 - i)
            if self._speed_profile[-i] <= 1.0 / 3.6:
                self._speed_profile[-i] = 1.0 / 3.6
        return self._speed_profile
    
    def generate_tvp(self):
        self._yaw_path_list = list(self._yaw_path_arr)
        self._speed_profile = []
        self._euc_dist = []
        for i in range(self._path_len):
            self._euc_dist.append(((self._x_initial - self._x_path_arr[i]) ** 2 + (self._y_initial - self._y_path_arr[i]) ** 2) ** 0.5)
        max_distance = self._euc_dist[-1]
        t_acc = self._target_speed / self._max_accel_decel
        t_sat = (max_distance / self._target_speed) - t_acc
        v_cur = 0.0
        t_cur = 0.0
        distance_ramp = 0.5 * t_acc * self._target_speed
        ramp_ratio = distance_ramp / max_distance
        idx_rmp_u = int(np.ceil(self._path_len * ramp_ratio))
        idx_rmp_d = int(np.floor(self._path_len - idx_rmp_u))
        for i in range(self._path_len - 1):
            if i <= idx_rmp_u:
                v_cur += self._max_accel_decel * (self._target_speed / distance_ramp)
            elif i <= idx_rmp_d:
                v_cur = self._target_speed
            else:
                v_cur -= self._max_accel_decel * (self._target_speed / distance_ramp)
            self._speed_profile.append(v_cur)
        self._speed_profile.append(0.0)
    
    def generate_tvp2(self):
        self._speed_profile = []
        self._euc_dist = []
        for i in range(self._path_len):
            self._euc_dist.append(((self._x_initial - self._x_path_arr[i]) ** 2 + (self._y_initial - self._y_path_arr[i]) ** 2) ** 0.5)
        v_sat = self._target_speed
        v_i = v_f = 0.0
        a_0 = self._max_accel_decel
        s_a = (v_sat ** 2 - v_i ** 2) / (2 * a_0)
        s_f = self._euc_dist[-1]
        s_b = s_f - ((v_f - (v_sat ** 2)) / (2 * -a_0))
        for i in range(self._path_len):
            if self._euc_dist[i] <= s_a:
                self._speed_profile.append(((2 * a_0 * self._euc_dist[i]) + (v_i ** 2)) ** 0.5)
            elif self._euc_dist[i] > s_a and self._euc_dist[i] < s_b:
                self._speed_profile.append(v_sat)
            elif self._euc_dist[i] >= s_b and self._euc_dist[i] < s_f:
                self._speed_profile.append((2 * - a_0 * (self._euc_dist[i] - s_b) + (v_sat ** 2)) ** 0.5)
            else:
                self._speed_profile.append(0.0)
            

    def get_current_path(self):
        return self._x_path_arr, self._y_path_arr, self._speed_profile


def config_dir():
    try:
        os.chdir('../../GenWaypoints/')
    except OSError:
        print "Waypoints Directory Not Found!"
        os.chdir(os.getcwd())
        os.chdir('../../')
        print "Creating \"GenWaypoints\" at ", os.getcwd()
        os.mkdir('GenWaypoints')
        print "Successfully Created Directory \"GenWaypoints\""
        os.chdir('GenWaypoints/')


def generate_file(path):
    try:
        os.remove('waypoints.txt')
    except:
        pass
    x_path, y_path, v_path = path.get_current_path()
    x_path = list(x_path)
    y_path = list(y_path)
    v_path = list(v_path)
    plan = [x_path, y_path, v_path]
    names_l = ['wpx', 'wpy', 'wpv']
    waypoints_file = open('waypoints.txt', "w")
    for i in range(len(plan)):
        tmp_l = plan[i]
        tmp_name = names_l[i]
        waypoints_file.write(tmp_name)
        waypoints_file.write(":\n")
        for j in range(len(tmp_l)):
            waypoints_file.write(str(tmp_l[j]))
            waypoints_file.write(" ")
        waypoints_file.write("\n")
        waypoints_file.write("-------------------------------------------")
        waypoints_file.write("\n")
    waypoints_file.close()
    return plan[0], plan[1], plan[2]


def plot_vp(vp):
    seq = []
    for i in range(len(vp)):
        seq.append(i + 1)
    _, _ = plt.subplots(1)
    plt.plot(seq, vp, 'g-')
    plt.grid(True)
    plt.xlabel("WP Sequence")
    plt.ylabel("Velocity [m/s]")


def plot_xy(x, y):
    _, _ = plt.subplots(1)
    plt.plot(x, y, 'b-')
    plt.grid(True)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")


def plot(x, y, v):
    plot_xy(x, y)
    plot_vp(v)
    plt.show()


def parse_args():
    args_list = list(sys.argv)
    if len(args_list) <= 1:
        req_mode = 'default'
        vp_mode = 'default'
    elif len(args_list) == 2:
        args_list.pop(0)
        req_mode = args_list[0]
        vp_mode = 'default'
    else:
        args_list.pop(0)
        req_mode = args_list[0]
        vp_mode = args_list[1]
    return req_mode, vp_mode


def main():
    mode, vp = parse_args()
    config_dir()
    if mode == 'default':
        is_default = True
    else:
        is_default = False
    path_obj = PathProps(is_default, mode)
    path_obj.generate_path()
    if vp == 'trap1':
        path_obj.generate_tvp()
    elif vp == 'trap2':
        path_obj.generate_tvp2()
    else:
        path_obj.generate_nvp()
    plan_x, plan_y, plan_v = generate_file(path_obj)
    plot(plan_x, plan_y, plan_v)


if __name__ == '__main__':
    main()