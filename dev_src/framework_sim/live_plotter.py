#! /usr/bin/env python

import matplotlib.pyplot as plt


def live_plot(cx, cy, x, y, target_ind):
    plt.cla()
    plt.plot(cx, cy, "-r", label="course")
    plt.plot(x, y, "ob", label="trajectory")
    plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
    plt.xlabel('X Global [m]')
    plt.ylabel('Y Global [m]')
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.title("Live Plotter")
    plt.pause(0.0001)