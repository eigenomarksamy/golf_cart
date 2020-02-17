#! /usr/bin/env python


import rospy
from GoToPointGoal import *


def main():
	try:
		x_pos = 0.0
		y_pos = 0.0
		psi_ori = 0.0
		x_vel = 0.0
		psi_rat = 0.0
		x_goal = input("Desired X: ")
		y_goal = input("Desired Y: ")
		yaw_goal = input("Desired YAW: ")
		goal = [x_goal, y_goal, yaw_goal]
		model_controller = GoToPointGoal("go_to_point_control", "audibot", x_pos, y_pos, psi_ori, x_vel, psi_rat, goal)

	except rospy.ROSInterruptException:
		print "Error Communication: Exception!"


if __name__ == '__main__':
	main()
