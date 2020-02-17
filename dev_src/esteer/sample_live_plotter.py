#! /usr/bin/python

import os
import rospy
from pp_feedback import live_plot

def main():
    rospy.init_node("whatever")
    r = rospy.Rate(10)
    x = y = []
    for i in range(100):                ### Stubbing Reference X Course Path
        x.append(i)
    for i in range(100):                ### Stubbing Reference Y Course Path
        y.append(i)
    for i in range(100):                ### Stubbing Current Path
        live_plot(x, y, x[i], y[i], i)  ### X, Y: Course Path, X[i], Y[i]: Current Point, i: Index In Reference Path
        r.sleep()

if __name__ == '__main__':
    main()