#! /usr/bin/python3


import sys
import numpy as np
sys.path.append("/home/oks/catkin_ws/src/framework_sim/clrn_ptracker/scripts/")
import cutils


class PurePursuit(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        input_steer = self._conv_rad_to_steer * input_steer_in_rad
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0
        self.vars.create_var('v_pre', 0.0)
        self.vars.create_var('t_pre', 0.0)
        self.vars.create_var('lot_err_pre', 0.0)
        self.vars.create_var('lot_err_intg_pre', 0.0)
        self.vars.create_var('sang_des', 0.0)
        self.vars.create_var('trot_des', 0.0)
        self.vars.create_var('trot_pre', 0.0)
        self.vars.create_var('brak_des', 0.0)
        self.vars.create_var('lot_pid_kp', 1.0)
        self.vars.create_var('lot_pid_ki', 0.1)
        self.vars.create_var('lot_pid_kd', 0.01)
        self.vars.create_var('lat_sta_ke', 0.1)
        self.vars.create_var('lat_lfw_k', 1.0)
        self.vars.create_var('wheelbase', 2.86)
        if self._start_control_loop:
            throttle_output = 0
            brake_output = 0
            Kp = self.vars.lot_pid_kp
            Ki = self.vars.lot_pid_ki
            Kd = self.vars.lot_pid_kd
            t_cur = t
            t_pre = self.vars.t_pre
            sample_t = t_cur - t_pre
            if sample_t == 0.0:
                sample_t += 0.1
            v_des = v_desired
            v_cur = v
            if v_des < 0.0:
                v_des = np.abs(v_des)
            v_err_cur = v_des - v_cur
            v_err_pre = self.vars.lot_err_pre
            v_err_pre_intg = self.vars.lot_err_intg_pre
            v_err_intg = v_err_pre_intg + v_err_cur * sample_t
            v_err_diff = (v_err_cur - v_err_pre) / sample_t
            des_acc = Kp * (v_err_cur) + Ki * (v_err_intg) + Kd * (v_err_diff)
            trot_pre = self.vars.trot_pre
            if des_acc > 0:
                trot_des = (np.tanh(des_acc) + 1) / 2
                if (trot_des - trot_pre) > 0.1:
                    trot_des = trot_pre + 0.1
            else:
                trot_des = 0
            self.vars.trot_des = trot_des
            throttle_output = self.vars.trot_des
            steer_output = 0
            x_cur = x
            y_cur = y
            yaw_cur = yaw
            Ke = self.vars.lat_sta_ke
            Lfc = self.vars.lat_lfw_k
            wheelbase = self.vars.wheelbase
            final_y = waypoints[-1][1]
            final_x = waypoints[-1][0]
            start_y = waypoints[0][1]
            start_x = waypoints[0][0]
            path_slope = (final_y - start_y) / (final_x - start_x)
            yaw_des = np.arctan2((final_y - start_y), (final_x - start_x))
            yaw_err = yaw_des - yaw_cur
            if yaw_err > np.pi:
                yaw_err -= 2 * np.pi
            elif yaw_err < -np.pi:
                yaw_err += 2 * np.pi
            lf = Ke * v_cur + Lfc
            sang_delta = np.arctan2(2.0 * wheelbase * np.sin(yaw_err) / lf, 1.0)
            sang_des = sang_delta
            if sang_des > np.pi:
                sang_des -= 2 * np.pi
            elif sang_des < -np.pi:
                sang_des += 2 * np.pi
            if sang_des > 1.0:
                sang_des = 1.0
            elif sang_des < -1.0:
                sang_des = -1.0
            self.vars.sang_des = sang_des
            steer_output    = self.vars.sang_des
            self.set_throttle(throttle_output)
            self.set_steer(steer_output)
            self.set_brake(brake_output)
        self.vars.v_previous = v_cur
        self.vars.t_pre = t_cur
        self.vars.trot_pre = self.vars.trot_des
        self.vars.lot_err_pre = v_err_cur
        self.vars.lot_err_intg_pre = v_err_intg
