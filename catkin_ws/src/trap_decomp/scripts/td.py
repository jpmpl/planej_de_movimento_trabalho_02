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

#OPENCV
import cv2 as cv

#SHAPELY
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

q0 = None
qf = None
theta = None
lrange = None
angle_min = None
angle_increment = None
k = 10
d = 0.5

class Cell:
    def __init__(self, trap_points, edges) -> None:
        self.trap_points = trap_points
        self.polygon = Polygon(trap_points)
        self.edges = edges
        self.is_covered = False

    def closest_vertice(self,p,rbt_shape):
        v_min = None
        obs_lim_start = None
        obs_lim_end  = None
        idx_min = None
        dist = inf
        for i in range(0,len(self.trap_points)):
            v = self.trap_points[i]
            if np.linalg.norm([v[0]-p[0],v[1]-p[1]]) < dist:
                dist = np.linalg.norm([v[0]-p[0],v[1]-p[1]])
                v_min = v
                idx_min = i
        if idx_min == 0:
            obs_lim_start = v_min[0]+0.3
            obs_lim_end = self.trap_points[2][0]-0.3
            v_min = np.array(v_min) + np.array([rbt_shape[0],rbt_shape[1]])
            end_x = self.trap_points[2][0]-rbt_shape[0]            
        if idx_min == 1:
            obs_lim_start = v_min[0]+0.3
            obs_lim_end = self.trap_points[2][0]-0.3
            v_min = np.array(v_min) + np.array([rbt_shape[0],-rbt_shape[1]])
            end_x = self.trap_points[2][0]-rbt_shape[0]
        if idx_min == 2:
            obs_lim_start = v_min[0]-0.3
            obs_lim_end = self.trap_points[2][0]+0.3
            v_min = np.array(v_min) + np.array([-rbt_shape[0],-rbt_shape[1]])
            end_x = self.trap_points[0][0]+rbt_shape[0]
        if idx_min == 3:
            obs_lim_start = v_min[0]-0.3
            obs_lim_end = self.trap_points[2][0]+0.3
            v_min = np.array(v_min) + np.array([-rbt_shape[0],rbt_shape[1]])
            end_x = self.trap_points[0][0]+rbt_shape[0]
        return v_min,end_x,obs_lim_start,obs_lim_end

    def distance_to_base(self,p,select_base):
        if not select_base:
            p0 = np.array(self.trap_points[0])
            p1 = np.array(self.trap_points[1])
        else:
            p0 = np.array(self.trap_points[2])
            p1 = np.array(self.trap_points[3])
        cross_prod = np.cross(p1-p0, p0-p)
        return np.linalg.norm(cross_prod)/np.linalg.norm(p1-p0), cross_prod > 0
    
    def direction_to_base(self,p):
        x = [self.trap_points[0][0],self.trap_points[1][0]]
        y = [self.trap_points[0][1],self.trap_points[1][1]]
        slope,offset = np.polyfit(x,y,1)
        grad = np.array([slope,-1])
        grad /= np.linalg.norm(grad)
        z = slope*p[0] - p[1] + offset
        if z > 0:
            return -grad
        elif z < 0:
            return grad
        else:
            return None

class World:
    
    def __init__(self, img, wrd_shape, robot_dim) -> None:
        self.img = img
        g_img = cv.cvtColor(img.copy(), cv.COLOR_BGR2GRAY)
        ret, self.g_img= cv.threshold(g_img,127,255,cv.THRESH_BINARY)
        self.scratch_img = img.copy()
        self.wrd_width = wrd_shape[0]
        self.wrd_height = wrd_shape[1]
        self.wrd_to_img_width_factor = img.shape[0]/wrd_shape[0]
        self.wrd_to_img_height_factor = img.shape[1]/wrd_shape[1]
        self.rbt_width = robot_dim[0]
        self.rbt_height = robot_dim[1]

    def transform_wrd_to_img(self, wrd_point):
        hom_trans = np.array([[1,0,30], [0,-1,30], [0,0,1]])
        point = np.array([[wrd_point[0]], [wrd_point[1]], [1]])
        img_point = np.dot(hom_trans,point)
        img_x = floor(img_point[0,0]*self.wrd_to_img_width_factor)
        img_y = floor(img_point[1,0]*self.wrd_to_img_height_factor)
        return np.array([img_x,img_y])
    
    def transform_img_to_wrd(self, img_point):
        img_x = img_point[0]/self.wrd_to_img_width_factor
        img_y = img_point[1]/self.wrd_to_img_height_factor
        hom_trans = np.array([[1,0,-30], [0,-1,30], [0,0,1]])
        point = np.array([[img_x], [img_y], [1]])
        wrd_point = np.dot(hom_trans,point)
        return np.array([wrd_point[0][0],wrd_point[1][0]])

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

def get_vertices(wrd):
    corner_img = np.zeros((wrd.scratch_img.shape[1], wrd.scratch_img.shape[0], 1), dtype = "uint8")
    gray = cv.cvtColor(wrd.scratch_img,cv.COLOR_BGR2GRAY)
    gray = np.float32(gray)
    dst = cv.cornerHarris(gray,15,7,0.04)
    #result is dilated for marking the corners, not important
    dst = cv.dilate(dst,None,iterations=5)
    dst = cv.erode(dst,None,iterations=2)
    # Threshold for an optimal value, it may vary depending on the image.
    corner_img[dst>0.001*dst.max()] = 255
    contours, hierarchy = cv.findContours(corner_img,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    
    img_v = []
    for c in contours:
        # calculate moments for each contour
        M = cv.moments(c)
        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        img_v.append([cX,cY])

    img_v.sort(key=lambda x: x[0])

    count = 0
    for v in img_v:
        (cX,cY) = (v[0],v[1])
        cv.circle(wrd.scratch_img, (cX, cY), 5, (0, 0, 255), -1)
        cv.putText(wrd.scratch_img, "v{}".format(count), (cX - 25, cY - 25),cv.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 4)
        count+=1
    
    # w = int(wrd.scratch_img.shape[1] * 0.5)
    # h = int(wrd.scratch_img.shape[0] * 0.5)
    # resized = cv.resize(wrd.scratch_img, (w,h), interpolation = cv.INTER_AREA)
    # cv.imshow('img',resized)
    # cv.imwrite('vertices.jpg',resized)
    # if cv.waitKey(0) & 0xff == 27:
    #     cv.destroyAllWindows()
    return img_v

def get_intersections(wrd, vertice):
    up = -1
    down = -1
    
    up_line = wrd.g_img[0:vertice[1],vertice[0]]
    for i in range(0,len(up_line)-10):
        if up_line[i] == 0 and i > up:
            up = i
    
    down_line = wrd.g_img[vertice[1]:,vertice[0]]
    for i in range(0+10,len(down_line)):
        if down_line[i] == 0:
            down = vertice[1]+i
            break

    if up == -1 and down == -1:
        return None,None
    elif up == -1:
        return [vertice[0],up],None
    elif down == -1:
        return None,[vertice[0],down]
    else:
        return [vertice[0],down],[vertice[0],up]

def get_cells(wrd,img_v):
    cells = dict()

    wrd_vs0 = wrd.transform_img_to_wrd(img_v[0])
    wrd_vs1 = wrd.transform_img_to_wrd(img_v[1])
    wrd_ve0 = wrd.transform_img_to_wrd(img_v[-1])
    wrd_ve1 = wrd.transform_img_to_wrd(img_v[-2])

    img_v = img_v[2:-2]

    wrd_v0 = wrd.transform_img_to_wrd(img_v[0])
    d,u = get_intersections(wrd,img_v[0])
    d0 = wrd.transform_img_to_wrd(d)
    u0 = wrd.transform_img_to_wrd(u)
    c1_trap = [wrd_vs0,wrd_vs1,u0,d0]
    c1_edges = dict()
    c1_edges[2] = [wrd_v0,u0]
    c1_edges[3] = [wrd_v0,d0]
    cells[1] = Cell(c1_trap,c1_edges)

    wrd_v1 = wrd.transform_img_to_wrd(img_v[1])
    d,u = get_intersections(wrd,img_v[1])
    d1 = wrd.transform_img_to_wrd(d)
    u1 = wrd.transform_img_to_wrd(u)
    c2_trap = [wrd_v0,u0,u1,d1]
    c2_edges = dict()
    c2_edges[1] = [wrd_v0,u0]
    c2_edges[4] = [wrd_v1,u1]
    c2_edges[5] = [wrd_v1,d1]
    cells[2] = Cell(c2_trap,c2_edges)
        
    wrd_v2 = wrd.transform_img_to_wrd(img_v[2])
    d,u = get_intersections(wrd,img_v[2])
    d2 = wrd.transform_img_to_wrd(d)
    u2 = wrd.transform_img_to_wrd(u)
    c4_trap = [wrd_v1,u1,u2,wrd_v2]
    c4_edges = dict()
    c4_edges[2] = [wrd_v1,u1]
    c4_edges[6] = [wrd_v2,u2]
    cells[4] = Cell(c4_trap,c4_edges)

    wrd_v3 = wrd.transform_img_to_wrd(img_v[3])#+np.array([0.5,-0.5])
    d,u = get_intersections(wrd,img_v[3])
    d3 = wrd.transform_img_to_wrd(d)
    u3 = wrd.transform_img_to_wrd(u)
    c3_trap = [d0,wrd_v0,wrd_v3,d3]
    c3_edges = dict()
    c3_edges[1] = [wrd_v0,d0]
    c3_edges[8] = [wrd_v3,d3]
    cells[3] = Cell(c3_trap,c3_edges)

    wrd_v4 = wrd.transform_img_to_wrd(img_v[4])
    d,u = get_intersections(wrd,img_v[4])
    d4 = wrd.transform_img_to_wrd(d)
    u4 = wrd.transform_img_to_wrd(u)
    c5_trap = [d1,wrd_v1,wrd_v4,d4]
    c5_edges = dict()
    c5_edges[2] = [wrd_v1,d1]
    c5_edges[7] = [wrd_v4,d4]
    cells[5] = Cell(c5_trap,c5_edges)

    wrd_v5 = wrd.transform_img_to_wrd(img_v[5])
    d,u = get_intersections(wrd,img_v[5])
    d5 = wrd.transform_img_to_wrd(d)
    u5 = wrd.transform_img_to_wrd(u)
    c6_trap = [wrd_v2,u2,wrd_v5,u5]
    c6_edges = dict()
    c6_edges[4] = [wrd_v2,u2]
    c6_edges[9] = [wrd_v5,u5]
    cells[6] = Cell(c6_trap,c6_edges)

    wrd_v6 = wrd.transform_img_to_wrd(img_v[6])
    c7_trap = [d4,wrd_v4,wrd_v6,wrd_v6]
    c7_edges = dict()
    c7_edges[5] = [wrd_v4,d4]
    cells[7] = Cell(c7_trap,c7_edges)

    wrd_v7 = wrd.transform_img_to_wrd(img_v[7])
    wrd_v8 = wrd.transform_img_to_wrd(img_v[8])
    d,u = get_intersections(wrd,img_v[8])
    d8 = wrd.transform_img_to_wrd(d)
    u8 = wrd.transform_img_to_wrd(u)
    c9_trap = [wrd_v5,u5,u8,wrd_v8]
    c9_edges = dict()
    c9_edges[6] = [wrd_v5,u5]
    c9_edges[11] = [wrd_v8,u8]
    cells[9] = Cell(c9_trap,c9_edges)
    c10_trap = [wrd_v7,wrd_v7,wrd_v8,d8]
    c10_edges = dict()
    c10_edges[11] = [wrd_v8,d8]
    cells[10] = Cell(c10_trap,c10_edges)
    
    wrd_v9 = wrd.transform_img_to_wrd(img_v[9])
    d,u = get_intersections(wrd,img_v[9])
    d9 = wrd.transform_img_to_wrd(d)
    u9 = wrd.transform_img_to_wrd(u)
    c8_trap = [d3,wrd_v3,wrd_v9,d9]
    c8_edges = dict()
    c8_edges[3] = [wrd_v3,d3]
    c8_edges[12] = [wrd_v9,d9]
    cells[8] = Cell(c8_trap,c8_edges)
    c11_trap = [d8,u8,u9,wrd_v9]
    c11_edges = dict()
    c11_edges[9] = [wrd_v8,u8]
    c11_edges[10] = [wrd_v8,d8]
    c11_edges[12] = [wrd_v9,u9]
    cells[11] = Cell(c11_trap,c11_edges)
    c12_trap = [d9,u9,wrd_ve0,wrd_ve1]
    c12_edges = dict()
    c12_edges[8] = [wrd_v9,d9]
    c12_edges[11] = [wrd_v9,u9]
    cells[12] = Cell(c12_trap,c12_edges)

    return cells

def obstacles(lrange):
    ex_lrange = np.hstack((lrange,lrange))
    lrange_local_mins = find_peaks(-ex_lrange, distance=20, prominence=0.3) # prominence=0.2 ?
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

    angles = []
    for idx in unique_lmin:
        ang = theta+angle_min+(idx%len(lrange))*angle_increment
        angles.append(ang)
    
    # angle = int(round((2*np.pi + np.arctan2(dir[1],dir[0]))%(2*np.pi)))
    return angles,unique_lmin,ex_lrange

def attraction_potential(q0,qgoal):
    di_threshold = 10
    a = 5
    di = np.linalg.norm(q0-qgoal)
    if di <= di_threshold:
        d_pot = a*(q0-qgoal)
    else:
        d_pot = di_threshold*a*(q0-qgoal)/di    
    return d_pot

def coverage(lrange,cells,cur_cell,q0,is_go_to_base,\
    is_vertical_movement, is_tangent_movement,is_up,\
    is_down,start_tangent,end_x_target,is_last_lap,\
    start_target,is_go_to_next_cell,next_cell_key,\
    next_cell_target,obs_lim_start,obs_lim_end):
    
    V_dir = None

    if is_go_to_base:
        start_target,end_x_target,obs_lim_start,obs_lim_end = cur_cell.closest_vertice(q0,[0.7,0.7])
        dist = np.linalg.norm(start_target-q0)
        #dir = (start_target-q0)/dist
        if dist > 0.005:
            V_dir = -attraction_potential(q0,start_target)

        # dist,side = cur_cell.distance_to_base(q0,False)
        # if dist > 1.5 and range_closest_obstacle() > 1.5:
        #     if side:
        #         V_dir = np.array([-1.0,0.0])
        #     else:
        #         V_dir = np.array([1.0,0.0])
        else:
            is_go_to_base = False
            is_vertical_movement = True

    if is_vertical_movement:
        angles,u_mins,lrng = obstacles(lrange)
        for i in range(0,len(angles)):
            ang = angles[i]
            p_obs = q0+lrng[u_mins[i]]*np.array([cos(ang),sin(ang)])
            if not is_up \
                and 180*ang/np.pi > -165 \
                and 180*ang/np.pi < -15 \
                and lrng[u_mins[i]] < 1.3 \
                and ((start_target[0] < end_x_target and p_obs[0]>obs_lim_start and p_obs[0]<obs_lim_end) \
                    or (start_target[0] > end_x_target and p_obs[0]<obs_lim_start and p_obs[0]>obs_lim_end)):
                if is_down:
                    is_down = False
                    is_vertical_movement = False
                    is_tangent_movement = True
                is_up = True
                break
            if not is_down \
                and 180*ang/np.pi > 15 \
                and 180*ang/np.pi < 165 \
                and lrng[u_mins[i]] < 1.3 \
                and ((start_target[0] < end_x_target and p_obs[0]>obs_lim_start and p_obs[0]<obs_lim_end) \
                    or (start_target[0] > end_x_target and p_obs[0]<obs_lim_start and p_obs[0]>obs_lim_end)):
                if is_up:
                    is_up = False
                    is_vertical_movement = False
                    is_tangent_movement = True
                is_down = True
                break

        if is_tangent_movement:
            start_tangent = q0

        if is_up:
            V_dir = np.array([0.0,1.0])
        if is_down:
            V_dir = np.array([0.0,-1.0])

    if is_tangent_movement:
        angles,u_mins,lrng = obstacles(lrange)
        for i in range(0,len(angles)):
            ang = angles[i]
            normal = -np.array([cos(ang), sin(ang)])
            if (180*ang/np.pi > -165 \
                and 180*ang/np.pi < -15) \
                or (180*ang/np.pi > 15 \
                and 180*ang/np.pi < 645):
                if lrng[u_mins[i]] < 1.0:
                    V_dir = normal
                elif lrng[u_mins[i]] > 1.6:
                    V_dir = -normal
                else:
                    tangent0 = np.array([-normal[1],normal[0]]) # 90º
                    tangent1 = np.array([normal[1],-normal[0]]) # -90º
                    if is_down:
                        if start_target[0] < end_x_target:
                            V_dir = tangent0
                        else:
                            V_dir = tangent1
                    if is_up:
                        if start_target[0] < end_x_target:
                            V_dir = tangent1
                        else:
                            V_dir = tangent0
                break
        
        if is_last_lap and abs(q0[0]-end_x_target) < 0.45:
            is_tangent_movement = False
            is_vertical_movement = False
            is_last_lap = False
            is_down = False
            is_up = False
            is_go_to_next_cell = True
        else:
            if abs(start_tangent[0]-q0[0]) > 1.0 or abs(q0[0]-end_x_target) < 0.45:
                is_tangent_movement = False
                is_vertical_movement = True
                if not is_last_lap and abs(q0[0]-end_x_target) < 0.45:
                    is_last_lap = True

    if is_go_to_next_cell:
        edge_center = None
        if not cur_cell.is_covered:
            cur_cell.is_covered = True
        if next_cell_key is None:
            min_dist = inf
            for key,edge in cur_cell.edges.items():
                edge_center = (np.array(edge[0])+np.array(edge[1]))/2
                if not cells[key].is_covered and np.linalg.norm(q0-edge_center) < min_dist:
                    min_dist = np.linalg.norm(q0-edge_center)
                    next_cell_key = key
        if edge_center is not None:
            if q0[0] < cur_cell.edges[next_cell_key][0][0]:
                next_cell_target = edge_center + np.array([1.0,0.0])
            else:
                next_cell_target = edge_center + np.array([-1.0,0.0])
        if next_cell_target is not None:
            if q0[1] > next_cell_target[1]+0.5:
                V_dir = np.array([0.0,-1.0])
            elif q0[1] < next_cell_target[1]-0.5:
                V_dir = np.array([0.0,1.0])
            else:
                V_dir = -attraction_potential(q0,next_cell_target)

            dist = np.linalg.norm(next_cell_target-q0)
            # if dist > 0.05:
            #     V_dir = -attraction_potential(q0,next_cell_target)
            if dist < 0.005:
                V_dir = None
                cur_cell = cells[next_cell_key]
                next_cell_key = None
                next_cell_target = None
                is_go_to_next_cell = False
                is_go_to_base = True
            

    return cur_cell,V_dir,is_go_to_base,is_vertical_movement,\
        is_tangent_movement, is_up,is_down,start_tangent,\
        end_x_target,is_last_lap,start_target,is_go_to_next_cell,\
        next_cell_key,next_cell_target,obs_lim_start,obs_lim_end
    

def init():
    global tangent_move, lrange
    rospy.init_node('td', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odometry_callback_robot)
    rospy.Subscriber('/base_scan', LaserScan, range_callback_robot)
    rate = rospy.Rate(20)
    vel_msg = Twist()

    while q0 is None or lrange is None:
        continue

    img = cv.imread("./catkin_ws/src/trap_decomp/worlds/td_world_2.png")
    wrd = World(img, [60,60], [1.5,1.5])
    
    img_v = get_vertices(wrd)
    cells = get_cells(wrd,img_v)

    cur_cell = None
    for key,cell in cells.items():
        if cell.polygon.contains(Point(q0[0],q0[1])):
            cur_cell = cell
            break

    alp = 0.2
    is_go_to_base = True
    is_vertical_movement = False
    is_tangent_movement = False
    is_up = False
    is_down = False
    start_tangent = None
    end_x_target = None
    is_last_lap = False
    start_target = None
    is_go_to_next_cell = False
    next_cell_key = None
    next_cell_target = None
    obs_lim_start = None
    obs_lim_end = None
    while not rospy.is_shutdown():
        if lrange is not None:
            cur_cell,V_dir,is_go_to_base,is_vertical_movement,is_tangent_movement,\
                is_up,is_down,start_tangent,end_x_target,is_last_lap,\
                start_target,is_go_to_next_cell,next_cell_key,\
                next_cell_target,obs_lim_start,obs_lim_end = \
                coverage(lrange.copy(),cells,cur_cell,q0,is_go_to_base,is_vertical_movement,\
                    is_tangent_movement,is_up,is_down,start_tangent,end_x_target,is_last_lap,\
                    start_target,is_go_to_next_cell,next_cell_key,\
                    next_cell_target,obs_lim_start,obs_lim_end)

            if V_dir is not None:    
                V = alp*V_dir
                if is_vertical_movement and (is_up or is_down):
                    V = 7*V
                        
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