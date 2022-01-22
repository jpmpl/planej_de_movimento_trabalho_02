#!/usr/bin/python3

#SYSTEM
from operator import is_not
import sys

#ROS
import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import *
from nav_msgs.msg import *
from math import *

#DATA STRUCTURE
import queue

#MATH
from tf.transformations import euler_from_quaternion
import numpy as np
from scipy.signal import argrelextrema, find_peaks, peak_prominences
import matplotlib.pyplot as plt

#OPENCV
import csv
import cv2 as cv


q0 = None
qf = None
theta = None
lrange = None
d = 0.5

class Prioritize:

    def __init__(self, priority, item):
        self.priority = priority
        self.item = item

    def __eq__(self, other):
        return self.priority == other.priority

    def __lt__(self, other):
        return self.priority < other.priority

class World:
    
    def __init__(self, img, wrd_shape, robot_dim) -> None:
        self.img = img
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
        wrd_point = np.dot(hom_trans,point)
        img_x = floor(wrd_point[0,0]*self.wrd_to_img_width_factor)
        img_y = floor(wrd_point[1,0]*self.wrd_to_img_height_factor)
        return np.array([img_x,img_y])

    def detect_collision(self,robot_pos,is_to_draw):
        start_point = self.transform_wrd_to_img(robot_pos+np.array([-self.rbt_width/2,self.rbt_height/2]))
        end_point = self.transform_wrd_to_img(robot_pos+np.array([self.rbt_width/2,-self.rbt_height/2]))        
        rect = self.img[start_point[1]:end_point[1],start_point[0]:end_point[0]]
        # cv.imshow('img', self.img)
        # cv.imshow('rect', rect)
        # cv.waitKey(0)
        # cv.destroyAllWindows()
        if np.sum(rect==0) > 0:
            return True
        else:
            if is_to_draw:
                print('Robot pos: {}'.format(robot_pos))
                print('Start point: {}'.format(start_point))
                print('End point: {}'.format(end_point))
                self.scratch_img = cv.rectangle(self.scratch_img,(start_point[0],start_point[1]),\
                    (end_point[0],end_point[1]),(255, 0, 0),2)
            return False

def odometry_callback_robot(data):
    global q0, theta, d
    orient = data.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    x0 = data.pose.pose.position.x #+ d * cos(theta)
    y0 = data.pose.pose.position.y #+ d * sin(theta)
    q0 = np.array([x0, y0])

def generate_rand_config(wrd):
    w = wrd.wrd_width/2 - wrd.rbt_width/2
    h = wrd.wrd_height/2 - wrd.rbt_height/2
    return [np.random.uniform(-w,w),np.random.uniform(-h,h)]

def generate_vertices(wrd,n_samples):
    vertices = []
    while len(vertices) < n_samples:
        rand_config = generate_rand_config(wrd)
        if not wrd.detect_collision(rand_config,True):
            vertices.append(rand_config)
    return np.array(vertices)

def has_colision(wrd, check_queue):
    if not check_queue.empty():
        start,end = check_queue.get()
        if np.linalg.norm((end-start)) < 0.25:
            return False
        mid = start+(end-start)/2
        if wrd.detect_collision(mid,False):
            # m = wrd.transform_wrd_to_img(mid)
            # wrd.scratch_img = cv.circle(wrd.scratch_img,\
            #     (m[0],m[1]),1,(0,0,255),2)
            # w = int(wrd.scratch_img.shape[1] * 0.5)
            # h = int(wrd.scratch_img.shape[0] * 0.5)
            # resized = cv.resize(wrd.scratch_img, (w,h), interpolation = cv.INTER_AREA)
            # cv.imshow('collision_img', resized)
            # cv.waitKey(0)
            # cv.destroyAllWindows()
            return True
        check_queue.put((start,mid))
        check_queue.put((mid,end))
        r1 = has_colision(wrd,check_queue)
        r2 = has_colision(wrd,check_queue)
        return r1 or r2
    return False

def is_linear_path_clear(wrd,start,end):
    check_queue = queue.Queue()
    check_queue.put((start,end))
    return not has_colision(wrd, check_queue)

def list_neighbors_candidates(v,vertices):
    return np.argsort(np.linalg.norm(vertices-v, axis=1))[1:]

def generate_edges(wrd,vertices,n_neighb):
    edges = []
    for v in vertices:
        neigh_cand_list = list_neighbors_candidates(v,vertices)
        neigh_list = []
        for i in neigh_cand_list:
            if is_linear_path_clear(wrd,v,vertices[i]):
                neigh_list.append(i)
                if len(neigh_list) == n_neighb:
                    break
        edges.append(neigh_list)
    return edges

def construct_prm(wrd, n_samples, n_neighb):
    vertices = generate_vertices(wrd,n_samples)
    w = int(wrd.scratch_img.shape[1] * 0.5)
    h = int(wrd.scratch_img.shape[0] * 0.5)
    resized = cv.resize(wrd.scratch_img, (w,h), interpolation = cv.INTER_AREA)
    cv.imshow('vertices_img', resized)
    cv.imwrite('sampled_wrd_vertices.jpg',resized)
    cv.waitKey(0)
    cv.destroyAllWindows()
    edges = generate_edges(wrd,vertices,n_neighb)
    for i in range(0,len(vertices)):
        for j in edges[i]:
            s = wrd.transform_wrd_to_img(vertices[i])
            e = wrd.transform_wrd_to_img(vertices[j])
            wrd.scratch_img = cv.line(wrd.scratch_img,\
                (s[0],s[1]),(e[0],e[1]),(0,255,0),2)
    resized = cv.resize(wrd.scratch_img, (w,h), interpolation = cv.INTER_AREA)
    cv.imshow('edges_img', resized)
    cv.imwrite('sampled_wrd_edges.jpg',resized)
    cv.waitKey(0)
    cv.destroyAllWindows()
    print('Done')
    return vertices, edges

def find_closest_vertice(wrd,vertices,q):
    for i in np.argsort(np.linalg.norm(vertices-q, axis=1)):
        if is_linear_path_clear(wrd,q,vertices[i]):
            return i
    return None

def attraction_potential(qgoal):
    di_threshold = 10
    a = 5
    di = np.linalg.norm(q0-qgoal)
    if di <= di_threshold:
        d_pot = a*(q0-qgoal)
    else:
        d_pot = di_threshold*a*(q0-qgoal)/di    
    return d_pot

def init():
    global tangent_move, lrange
    rospy.init_node('prd', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odometry_callback_robot)
    rospy.Subscriber('/base_scan', LaserScan, range_callback_robot)
    rate = rospy.Rate(20)
    vel_msg = Twist()

    img = cv.imread("./catkin_ws/src/probabilistic_roadmap/worlds/prd_world.png")
    wrd = World(img, [60,60], [1.5,1.5])

    is_goal_valid = False
    while not is_goal_valid:
        if sys.version_info.major == 2:
            qfx, qfy = raw_input('Digite as coordenadas do alvo (x,y): ').split()
        elif sys.version_info.major == 3:
            qfx, qfy = input('Digite as coordenadas do alvo (x,y): ').split()
        
        qfx, qfy = [float(i) for i in [qfx, qfy]]
        qf = np.array([qfx,qfy])
        if wrd.detect_collision(qf,False):
            print('Collision detected: invalid input. Try another goal.')
        else:
            is_goal_valid = True

    n_samples = 200
    n_neighb = 5
    vertices,edges = construct_prm(wrd, n_samples, n_neighb)

    while q0 is None:
        continue

    idx_0 = find_closest_vertice(wrd,vertices,q0)
    idx_f = find_closest_vertice(wrd,vertices,qf)
    vert_f = vertices[idx_f]

    fronteir = queue.PriorityQueue()
    fronteir.put(Prioritize(0,idx_0))
    cost_so_far = dict()
    cost_so_far[idx_0] = 0
    came_from = dict()
    came_from[idx_0] = None

    while not fronteir.empty():
        idx_c_vert = fronteir.get().item

        if idx_c_vert == idx_f:
            break

        for next in edges[idx_c_vert]:
            c_vert = vertices[idx_c_vert]
            n_vert = vertices[next]
            new_cost = cost_so_far[idx_c_vert] + np.linalg.norm(c_vert-n_vert)
            if next not in cost_so_far.keys() or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + abs(vert_f[0]-n_vert[0]) + abs(vert_f[1]-n_vert[1])
                fronteir.put(Prioritize(priority,next))
                came_from[next] = idx_c_vert

    if idx_f not in came_from.keys():
        print('Path not found')
        raise

    backtrace = []
    backtrace.append(qf)
    idx_c = idx_f
    backtrace.append(vertices[idx_c])
    while came_from[idx_c] is not None:
        idx_c = came_from[idx_c]
        backtrace.append(vertices[idx_c])

    ep = 0.005
    alp = 0.1
    cur_target = backtrace.pop()
    while not rospy.is_shutdown() and np.linalg.norm(q0-qf) > ep:
        if len(backtrace) > 0 and np.linalg.norm(q0-cur_target) < 0.5:
            cur_target = backtrace.pop()

        d_atr_pot = attraction_potential(cur_target)
        d_pot = d_atr_pot
        V = - alp*(d_pot)

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