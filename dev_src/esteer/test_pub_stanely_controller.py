'''
Created on Apr 27, 2018

@author: mloay
'''
from _abcoll import Mapping
import stanely

if __name__ == '__main__':
    pass

###################################################### Import ##############################################################
import os
import sys
import numpy as np
import math

sys.path.append('../')
sys.path.append('../Algos')
sys.path.append('../helper')
from stanely import Satnely as Satnely 
from waypoints_parser import GenWP
##############################################################################################################################

# index =0;
# wp_obj = GenWP()
# x,y = wp_obj.getWaypointsArray()
# # xf_g,yf_g = wp_obj.getlastPoint()
stanely_controller = Satnely(0.5,1.0)
# dirpath = os.getcwd()
# print(dirpath)
# dirpath = dirpath +'/Algos'
# print(dirpath)
# 
# ##TEST 1 :Reading data from waypoint parser
# ret = False
# print(x)
# print(y)
# x_prev = x[0];
# y_prev = y[0];
# 
# x_curr = x[1];
# y_curr = y[1];
# 
# xy_way = np.array([[x_prev,y_prev],[x_curr, y_curr]])
# xx = np.array(xy_way)[:, :2]
# 
# ret = (xy_way[0][0] == xx[0][0])
# ret &= (xy_way[0][1] == xx[0][1])    
# 
# if(ret == True):
#     print("Test 1 passed")
# else:
#     print("Test 1 Failed")
# 
# # ##TEST 2: Check the nearest way point(even if repeated)
# ret = False
# wx,wy,wv = wp_obj.choose_waypoints(8.75, -4.335)
# index_g =wp_obj.getIndex()
# print(index_g)
# ret = (index_g == 6)
# ret &= (wx == 9.0 and wy==-4.335)
#  
# wx,wy,wv = wp_obj.choose_waypoints(8.75, -4.335)
# index_g =wp_obj.getIndex()
# print(index_g)
# ret &= (index_g == 6)
# ret &= (wx == 9.0 and wy==-4.335)
#  
# 
# wx,wy,wv = wp_obj.choose_waypoints(12.3, -4.335)
# index_g =wp_obj.getIndex()
# print(index_g)
# ret &= (index_g == 8)
# ret &= (wx == 12.0 and wy==-4.335)
# 
#  
# wx,wy,wv = wp_obj.choose_waypoints(12.3, -4.335)
# index_g =wp_obj.getIndex()
# print(index_g)
# ret &= (index_g == 8)
# ret &= (wx == 12.0 and wy==-4.335)
#  
# if(ret == True):
#     print("Test 2 passed")
# else:
#     print("Test 2 Failed")
#      
# ##TEST 3: The functional output of the controller
# 
# stanely_controller.reset()
# stanely_controller.setVehicleData(4.0, 5.0, 1.0, math.radians(30.0))
# stanely_controller.setWayPoints(6.0, 9.0, 2.0, 3.0)
# steer_output = stanely_controller.controller_update()
# print(steer_output)

#TEST_4:Mapping
output = stanely_controller.mapping(-25, -35.0, 35, -1.0, 1.0)
print(output)


##############################################################################################################################
