#! /usr/bin/env python

import rospy
import os
import sys
import math



from dbw_mkz_road.msg import ControlInput
from dbw_mkz_road.msg import Feedback
from __builtin__ import False


dirpath = os.path.dirname(os.path.abspath(__file__))
dirAlgo = dirpath +'/Algos'
dirHelper = dirpath +'/helper'

sys.path.append(dirpath)
sys.path.append(dirAlgo)
sys.path.append(dirHelper)

from wparser import GenWP
from stanely import Satnely as Satnely 
from pid import PID
from pp_feedback import live_plot


def feedback_callback(feedback_msg):
    global current_pos_x_g, current_pos_y_g,current_yaw_g,vel_g,current_g_ts
    current_pos_x_g = feedback_msg.global_x
    current_pos_y_g = feedback_msg.global_y
    current_yaw_g = feedback_msg.global_yaw
    current_g_ts = feedback_msg.timestamp
    vel_g = feedback_msg.global_v

def main():
    #Initialize variables
    global current_pos_x_g, current_pos_y_g,current_yaw_g,vel_g,current_g_ts,prev_g_ts,xf_g,yf_g,xway_arr,yway_arr
    current_pos_x_g =0.0
    current_pos_y_g =0.0
    current_yaw_g   =0.0
    vel_g           =0.0
    current_g_ts    =0.0
    prev_g_ts       =0.0
    xf_g            =0.0
    yf_g            =0.0

    #Initialize path
    wp_obj = GenWP(dirpath)
    xway_arr,yway_arr = wp_obj.getWaypointsArray()
    
    #Initialize PID Controller
    pidController = PID(Kp=1.0, Ki=1.0, Kd=0.01,windupVal=80);
    pidController.output_limits = (0.0,100.0);

    #Initialize Stanely Controller
    stanely_controller = Satnely(0.35,1.0)
    
    
    #Initialze ROS-Node
    rospy.init_node('pub_stanely_controller')
    stanely_ctrl_pub = rospy.Publisher('/vehicle/control_cmd', ControlInput, queue_size=10)
    r = rospy.Rate(10)
    control_msg = ControlInput()
    
    while not rospy.is_shutdown():
        #Receive feed-back message
        control_msg.timestamp = rospy.Time.now()
        rospy.Subscriber('/vehicle/feedback', Feedback, feedback_callback)
        current_pos_x_g = current_pos_x_g + 1.5*math.cos(current_yaw_g)
        current_pos_y_g = current_pos_y_g + 1.5*math.sin(current_yaw_g)

        current_g_ts = rospy.get_time()
        cyclic_time = float(current_g_ts) - float(prev_g_ts)
        ts = float(current_g_ts)
        if cyclic_time == 0.0:
            cyclic_time = 0.01
        prev_g_ts = current_g_ts


        #choose way-point according to current vehicle position
        xf_g,yf_g = wp_obj.getlastPoint()
        wx,wy,wv  = wp_obj.choose_waypoints(current_pos_x_g, current_pos_y_g)
        
        #Apply PID-controller on velocity
        vel_out = pidController.update(wv,vel_g,cyclic_time)
        
        #update stanely with new waypoints and new position and velocity
        stanely_controller.setVehicleData(current_pos_x_g, current_pos_y_g, vel_g, current_yaw_g)
        stanely_controller.setWayPoints(xf_g,yf_g,wx,wy)
        steer_output = stanely_controller.controller_update()
        
        
        #Update control message and send it
        control_msg.cmd_velocity = vel_out/100.0  ## No upper limit
        control_msg.cmd_heading = steer_output;   ## from 1.0 to -1.0, 0.0 is the center     
        stanely_ctrl_pub.publish(control_msg)
        
        #Print Debug Data
        index = wp_obj.getIndex()

        r.sleep()


if __name__ == '__main__':
    main()