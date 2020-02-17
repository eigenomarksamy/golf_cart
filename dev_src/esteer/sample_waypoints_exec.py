#! /usr/bin/env python

import sys
sys.path.append('../helper')
import waypoints_parser


def main():
    wp_obj = waypoints_parser.GenWP()
    wp = wp_obj.execute('wp_np')
    wp_x = []
    wp_y = []
    wp_v = []
    for i in range(wp.shape[0]):
        wp_x.append(wp[i][0])
    for i in range(wp.shape[0]):
        wp_y.append(wp[i][1])
    for i in range(wp.shape[0]):
        wp_v.append(wp[i][2])
    print " "
    print "For x-waypoints: "
    print wp_x
    print "--------------------------------------------------------------------------------"
    print " "
    print "For y-waypoints: "
    print wp_y
    print "--------------------------------------------------------------------------------"
    print " "
    print "For v-waypoints: "
    print wp_v
    print "--------------------------------------------------------------------------------"


if __name__ == '__main__':
    main()