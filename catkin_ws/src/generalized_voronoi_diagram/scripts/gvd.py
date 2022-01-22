#!/usr/bin/python3

#SYSTEM
from configparser import ExtendedInterpolation
from operator import is_not
from re import L
import sys

#ROS
import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import *
from nav_msgs.msg import *
from math import *

#DATA STRUCTURE
from queue import PriorityQueue

#MATH
from tf.transformations import euler_from_quaternion
import numpy as np
from scipy.signal import argrelextrema, find_peaks, peak_prominences
from scipy.linalg import null_space
import matplotlib.pyplot as plt
from findpeaks import findpeaks
import pandas as pd

tangent_move = False
q0 = None
qf = None
theta = None
lrange = None
angle_min = None
angle_increment = None
k = 10
d = 0.5

class vertice:
    def __init__(self, center, directions):
        self.center = center
        self.directions = directions

    def update_connected_vertice(self,dir_index,con_vertice):
        self.directions[dir_index][1] = con_vertice
    
    def unexplored_directions(self):
        unex_dir = []
        for i in range(0,len(self.directions)):
            if self.directions[i][1] is None:
                unex_dir.append((i,self.directions[i][0]))
        return unex_dir
    
    def is_in_vertice(self, p):
        ep = 2*0.5
        return np.linalg.norm(self.center-p) <= ep
    
    def find_destination_dir(self, vertice_idx):
        for i in range(0,len(self.directions)):
            if self.directions[i][1] == vertice_idx:
                return i,self.directions[i][0]
        raise

def odometry_callback_robot(data):
    global q0, theta, k, d
    orient = data.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    x0 = data.pose.pose.position.x #+ d * cos(theta)
    y0 = data.pose.pose.position.y #+ d * sin(theta)
    q0 = np.array([x0, y0])

def range_callback_robot(data):
    global lrange, angle_min, angle_increment
    lrange = np.array(data.ranges)
    angle_min = data.angle_min
    angle_increment = data.angle_increment

def get_points(lrange, local_mins, q0):
    q_list=[]
    for i in local_mins:
        mi_i = theta+angle_min+i*angle_increment
        q = q0 + lrange[i]*np.array([cos(mi_i),sin(mi_i)])
        q_list.append(q)
    return q_list

def sort_anticlockwise(q_list, q0):
    q_list.sort(key=lambda x: np.arctan2(x[1]-q0[1],x[0]-q0[0]))
    return q_list

def line_fit(p1, p2):
    x = [p1[0],p2[0]]
    y = [p1[1],p2[1]]
    return np.polyfit(x,y,1)

def get_candidates(lrange, local_mins, q0):
    assert len(local_mins) >= 2

    cand_list = []
    q_list = get_points(lrange, local_mins, q0)
    q_list = sort_anticlockwise(q_list, q0)
    len_q_list = len(q_list)
    if len_q_list > 2:
        for i in range(0,len_q_list):
            slope,offset = line_fit(q_list[i%len_q_list],q_list[(i+1)%len_q_list])
            grad = np.array([slope,-1])
            grad /= np.linalg.norm(grad)
            z = slope*q0[0] - q0[1] + offset
            if z > 0:
                cand_list.append(-grad)
            elif z < 0:
                cand_list.append(grad)
            else:
                cand_list.append(None)
    else:
        slope,offset = line_fit(q_list[0],q_list[1])
        grad = np.array([slope,-1])
        grad /= np.linalg.norm(grad)
        z = slope*q0[0] - q0[1] + offset
        if z > 0:
            cand_list.append(-grad)
        elif z < 0:
            cand_list.append(grad)
        else:
            raise
    return cand_list

def choose_path(lrange,local_mins, history, q0, vertice_list, search_stack, last_ver_idx, last_sel_dir_index, is_extreme_vertice):
    cur_ver_idx = None

    for i in range(0,len(vertice_list)): # check if vertice already exists
        if vertice_list[i].is_in_vertice(q0):
            cur_ver_idx = i
            break

    if cur_ver_idx is None: # new vertice found
        cand_list = get_candidates(lrange,local_mins,q0)

        if is_extreme_vertice:
            assert len(cand_list) == 1
            directions = [[-cand_list[0], None]]
        else:
            directions = []
            for cand in cand_list:
                directions.append([cand,None])

        vertice_list.append(vertice(q0,directions))
        cur_ver_idx = len(vertice_list)-1

    assert cur_ver_idx is not None

    search_stack.append(cur_ver_idx)

    unex_dirs = vertice_list[cur_ver_idx].unexplored_directions()
    if is_extreme_vertice:
        assert len(unex_dirs) == 1
    if not history[history.isD1==False].empty:
        last_reg = history[history.isD1==False].iloc[-1]
    else:
        last_reg = history.iloc[-1]
    for unex_dir in unex_dirs:
        cand = unex_dir[1]
        criteria = 0.5
        l = np.array([last_reg.v_x,last_reg.v_y])
        if last_ver_idx is not None and np.linalg.norm(-cand-l) < criteria:
            vertice_list[cur_ver_idx].update_connected_vertice(unex_dir[0],last_ver_idx)
            vertice_list[last_ver_idx].update_connected_vertice(last_sel_dir_index,cur_ver_idx)
            break
    
    if is_extreme_vertice:
        last_ver_idx = cur_ver_idx
        search_stack.pop()
        dest_ver_idx = search_stack.pop()
        last_sel_dir_index, dir = vertice_list[cur_ver_idx].find_destination_dir(dest_ver_idx)
        return dir, True, False, vertice_list,\
            search_stack, last_ver_idx, last_sel_dir_index,last_reg, 'vertice_decision'
    else:
        cur_unex_dirs = vertice_list[cur_ver_idx].unexplored_directions()
        if len(cur_unex_dirs) > 0:
            last_ver_idx = cur_ver_idx
            last_sel_dir_index = cur_unex_dirs[0][0]
            return cur_unex_dirs[0][1], True, False, vertice_list, search_stack, last_ver_idx, last_sel_dir_index, last_reg, 'vertice_decision'
        else:
            last_ver_idx = cur_ver_idx
            search_stack.pop()
            dest_ver_idx = search_stack.pop()
            last_sel_dir_index, dir = vertice_list[cur_ver_idx].find_destination_dir(dest_ver_idx)
            return dir, True, False, vertice_list,\
                 search_stack, last_ver_idx, last_sel_dir_index, last_reg, 'vertice_decision'

def get_velocity_direction(lrange, history, q0, vertice_list, search_stack, last_ver_idx, last_sel_dir_index):
    ep = 0.05

    if not history[history.isD1==False].empty:
        last_reg = history[history.isD1==False].iloc[-1]
    else:
        last_reg = history.iloc[-1]
    
    if len(vertice_list) > 0:
        if vertice_list[last_ver_idx].is_in_vertice(q0):
            return vertice_list[last_ver_idx].directions[last_sel_dir_index][0], True, False,\
                vertice_list, search_stack, last_ver_idx, last_sel_dir_index, last_reg, 'in_vertice'
    
    ex_lrange = np.hstack((lrange,lrange))
    lrange_local_mins = find_peaks(-ex_lrange, distance=20, prominence=0.5) # prominence=0.2 ?
    lrange_lmins = lrange_local_mins[0]

    unique_lmin = []
    for lmin_l in lrange_lmins:
        add_lmin = True
        for lmin_u in unique_lmin:
            if abs(lmin_l%len(lrange) - lmin_u%len(lrange)) < 10:
                add_lmin = False
                break
        if add_lmin:
            unique_lmin.append(lmin_l)
    
    unique_lmin.sort(key=lambda x: ex_lrange[x])
    multiple_min = [ i for i in unique_lmin if abs(ex_lrange[i]-ex_lrange[unique_lmin[0]]) < ep]
    if len(multiple_min) > 2:
        return choose_path(ex_lrange,multiple_min, history, q0,\
            vertice_list, search_stack, last_ver_idx, last_sel_dir_index, False)

    if len(unique_lmin) > 0:
        min_fst = unique_lmin[0]
        mi_fst = theta+angle_min+min_fst*angle_increment-pi
        di_fst = np.array([cos(mi_fst), sin(mi_fst)])
        if len(lrange_local_mins) > 1:
            min_snd = unique_lmin[1]
            if abs(ex_lrange[min_fst]-ex_lrange[min_snd]) < ep:
                cand = get_candidates(ex_lrange,unique_lmin[0:2],q0)
                assert len(cand) == 1
                cand1 = cand[0]
                cand2 = -cand[0]
                if ex_lrange[min_fst] > 1.5 and ex_lrange[min_snd] > 1.5:
                    l = np.array([last_reg.v_x,last_reg.v_y])
                    if np.linalg.norm(cand1 - l) < np.linalg.norm(cand2 - l):
                        return cand1, False, False, vertice_list, search_stack, last_ver_idx, last_sel_dir_index,last_reg, 'tangent_decision'
                    else:
                        return cand2, False, False, vertice_list, search_stack, last_ver_idx, last_sel_dir_index,last_reg, 'tangent_decision'
                else:
                    return choose_path(ex_lrange,multiple_min, history, q0,\
                        vertice_list, search_stack, last_ver_idx, last_sel_dir_index, True)
        return di_fst, False, True, vertice_list, search_stack, last_ver_idx, last_sel_dir_index, last_reg, 'closest'

def init():
    global tangent_move, lrange
    rospy.init_node('gvd', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odometry_callback_robot)
    rospy.Subscriber('/base_scan', LaserScan, range_callback_robot)
    rate = rospy.Rate(20)
    vel_msg = Twist()

    while q0 is None:
        continue

    df = pd.DataFrame(columns=['q0_x','q0_y','v_x','v_y','v_angle','isGVD'])
    df = df.append({'q0_x':q0[0],'q0_y':q0[1],'v_x':0,'v_y':0,'v_angle':None,\
        'isVertice': None,'isD1':None,'isGVD':None}, ignore_index=True)
    #last_point = df.iloc[-1]

    ep = 0.005
    alp = 0.2
    vertice_list = []
    search_stack = []
    last_ver_idx = None
    last_sel_dir_index = None
    while not rospy.is_shutdown():
        if lrange is not None:

            tangent_move = False
            V_dir,isVertice,isD1,vertice_list, search_stack,last_ver_idx,last_sel_dir_index,last_regis,decision\
                = get_velocity_direction(lrange, df, q0, vertice_list, search_stack, last_ver_idx, last_sel_dir_index)
            V = alp*V_dir
            print("v_x: {} v_y: {} | lv_x: {} lv_y: {} | Decision: {}".format(V_dir[0],V_dir[1],last_regis.v_x,last_regis.v_y, decision))
            
            df = df.append({'q0_x':q0[0],'q0_y':q0[1],'v_x':V_dir[0],'v_y':V_dir[1],'v_angle':np.arctan2(V_dir[1],V_dir[0]),\
                'isVertice': isVertice,'isD1':isD1,'isGVD':tangent_move}, ignore_index=True)
            
            # Omnidirectional robot
            vel_msg.linear.x = V[0]
            vel_msg.linear.y = V[1]
            
            # Diff robot
            #vel_msg.linear.x = cos(theta)*V[0]+sin(theta)*V[1]
            #vel_msg.angular.z = (-sin(theta)*V[0]+cos(theta)*V[1])/d

            rate.sleep()
            pub.publish(vel_msg)
    
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass