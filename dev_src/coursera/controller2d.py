#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
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
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
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

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
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
        self.vars.create_var('lat_sta_ke', 0.5)
        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            throttle_output = 0
            brake_output = 0
            Kp = self.vars.lot_pid_kp
            Ki = self.vars.lot_pid_ki
            Kd = self.vars.lot_pid_kd
            t_cur = t
            t_pre = self.vars.t_pre
            sample_t = t_cur - t_pre
            v_des = v_desired
            v_cur = v
            v_err_cur = v_des - v_cur
            v_err_pre = self.vars.lot_err_pre
            v_err_pre_intg = self.vars.lot_err_intg_pre
            v_err_intg = v_err_pre_intg + v_err_cur * sample_t
            v_err_diff = (v_err_cur - v_err_pre) / sample_t
            des_acc = Kp * (v_err_cur) + Ki * (v_err_intg) + Kd * (v_err_diff)
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.''
            trot_pre = self.vars.trot_pre
            if des_acc > 0:
                trot_des = (np.tanh(des_acc) + 1) / 2
                if (trot_des - trot_pre) > 0.1:
                    trot_des = trot_pre + 0.1
            else:
                trot_des = 0
            self.vars.trot_des = trot_des
            throttle_output = self.vars.trot_des

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            steer_output = 0
            x_cur = x
            y_cur = y
            yaw_cur = yaw
            Ke = self.vars.lat_sta_ke
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
            xy_cur = np.array([x_cur, y_cur])
            crosstrack_err = np.min(np.sum((xy_cur - np.array(waypoints)[:, :2])**2, axis=1))
            yaw_crosstrack = np.arctan2(y_cur - start_y, x_cur - start_x)
            yaw_instref = yaw_des - yaw_crosstrack
            if yaw_instref > np.pi:
                yaw_instref -= 2 * np.pi
            elif yaw_instref < -np.pi:
                yaw_instref += 2 * np.pi
            if yaw_instref > 0:
                crosstrack_err = abs(crosstrack_err)
            else:
                crosstrack_err = -abs(crosstrack_err)
            yaw_err_crosstrack = np.arctan(Ke * crosstrack_err / v_cur)
            sang_des = yaw_err_crosstrack + yaw_err
            if sang_des > np.pi:
                sang_des -= 2 * np.pi
            elif sang_des < -np.pi:
                sang_des += 2 * np.pi
            if sang_des > 1.22:
                sang_des = 1.22
            elif sang_des < -1.22:
                sang_des = -1.22
            self.vars.sang_des = sang_des
            # Change the steer output with the lateral controller. 
            steer_output    = self.vars.sang_des

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v_cur  # Store forward speed to be used in next step
        self.vars.t_pre = t_cur
        self.vars.trot_pre = self.vars.trot_des
        self.vars.lot_err_pre = v_err_cur
        self.vars.lot_err_intg_pre = v_err_intg
