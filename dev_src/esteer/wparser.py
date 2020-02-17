#! /usr/bin/env python


import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

class GenWP:

    def __init__(self, dirpath, pathtype='wp_xyv'):
        self.course_txt = ["wpx:\n", "wpy:\n", "wpv:\n"]
        self.waypoints_x = []
        self.waypoints_y = []
        self.waypoints_v = []
        self._index_g =0
        self.set_dir(dirpath)
        self.waypoints_x,self.waypoints_y,self.waypoints_v =self.execute(pathtype)
        self._waypoints_length =len(self.waypoints_x)

    def parse_path(self):
        try:
            os.chdir(self.get_dir())
            self.gen_wp_path = "../GenWaypoints/waypoints.txt"
            self.course_file = open(self.gen_wp_path, 'r')
            self.lines = self.course_file.readlines()
            for line in range(len(self.lines) - 1):
                if self.lines[line] == self.course_txt[0]:
                    for i in self.lines[line + 1].split():
                        self.waypoints_x.append(float(i))
                elif self.lines[line] == self.course_txt[1]:
                    for i in self.lines[line + 1].split():
                        self.waypoints_y.append(float(i))
                elif self.lines[line] == self.course_txt[2]:
                    for i in self.lines[line + 1].split():
                            self.waypoints_v.append(float(i))
        except OSError:
            print("No File or Directory!")
        return [self.waypoints_x, self.waypoints_y, self.waypoints_v]


    def get_wp_props(self, wp_conc_list):
        self.wp_x = wp_conc_list[0]
        self.wp_y = wp_conc_list[1]
        self.wp_v = wp_conc_list[2]
        self.wp = list(zip(self.wp_x, self.wp_y, self.wp_v))
        self.wp_np = np.array(self.wp)
        self.wp_distance = []
        for i in range(1, self.wp_np.shape[0]):
            self.wp_distance.append(np.sqrt((self.wp_np[i, 0] - self.wp_np[i-1, 0])**2 +
                                            (self.wp_np[i, 1] - self.wp_np[i-1, 1])**2))
        self.wp_distance.append(0)
        return [self.wp_x, self.wp_y, self.wp, self.wp_np, self.wp_distance]


    def plotter(self, x, y):
        plt.plot(x, y, 'g-')
        plt.show()


    def execute(self, return_type = 'list', plot = False):
        self.wp_conc = self.parse_path()
        self.wp_props = self.get_wp_props(self.wp_conc)
        if plot:
            self.plotter(self.wp_props[0], self.wp_props[1])
        if return_type == 'wp_list':
            return self.wp_props
        elif return_type == 'wp_xy':
            return self.wp_props[0], self.wp_props[1]
        elif return_type == 'wp_xyv':
            return self.wp_props[0], self.wp_props[1] , self.wp_conc[2] 
        elif return_type == 'wp_dic':
            return self.wp_props[2]
        elif return_type == 'wp_np':
            return self.wp_props[3]
        else:
            return self.wp_props
        
    def choose_waypoints(self,current_pos_x_g, current_pos_y_g):
        '''
        Find the index which crroespondes to the minmum distance between
        vehicle position and waypoints
        '''
        dx = [current_pos_x_g - self.waypoints_x[icx] for icx in range(self._index_g,self._waypoints_length)]
        dy = [current_pos_y_g - self.waypoints_y[icy] for icy in range(self._index_g,self._waypoints_length)]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx,idy) in zip(dx,dy)]
        temp_index = d.index(min(d))
        self._index_g = self._index_g + temp_index
        
        wx = self.waypoints_x[self._index_g]
        wy = self.waypoints_y[self._index_g]
        wv = self.waypoints_v[self._index_g]
        return wx,wy,wv;
    
    def getlastPoint(self):
        wx = self.waypoints_x[-1]
        wy = self.waypoints_y[-1]
        return wx,wy;
    
    def getWaypointsArray(self):
        wx = self.waypoints_x
        wy = self.waypoints_y
        return wx,wy;
    
    def getIndex(self):
        return self._index_g


    def set_dir(self, rsp_dir):
        self.req_dir = rsp_dir


    def get_dir(self):
        return self.req_dir


def main():
    wp_obj = GenWP()
    wp = wp_obj.execute('wp_np')


if __name__ == '__main__':
    main()