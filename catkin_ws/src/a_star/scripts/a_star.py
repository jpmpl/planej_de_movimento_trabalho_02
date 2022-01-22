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
from queue import PriorityQueue

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

class Cell:

    def __init__(self, row, col, value):
        self.row = row
        self.col = col
        self.value = value
    
    def neighbors(self, grid):
        neighbors = []
        if self.row+1 <= grid.shape[0]:
            neighbors.append(grid[self.row+1,self.col])
        if self.row-1 > 0:
            neighbors.append(grid[self.row-1,self.col])
        if self.col+1 <= grid.shape[1]:
            neighbors.append(grid[self.row,self.col+1])
        if self.col-1 > 0:
            neighbors.append(grid[self.row,self.col-1])
        return neighbors

class Prioritize:

    def __init__(self, priority, item):
        self.priority = priority
        self.item = item

    def __eq__(self, other):
        return self.priority == other.priority

    def __lt__(self, other):
        return self.priority < other.priority

def odometry_callback_robot(data):
    global q0, theta, d
    orient = data.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    x0 = data.pose.pose.position.x + d * cos(theta)
    y0 = data.pose.pose.position.y + d * sin(theta)
    q0 = np.array([x0, y0])

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
    rospy.init_node('a_star', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odometry_callback_robot)
    rate = rospy.Rate(20)
    vel_msg = Twist()

    if sys.version_info.major == 2:
        qfx, qfy = raw_input('Digite as coordenadas do alvo (x,y): ').split()
    elif sys.version_info.major == 3:
        qfx, qfy = input('Digite as coordenadas do alvo (x,y): ').split()
    
    qfx, qfy = [float(i) for i in [qfx, qfy]]
    qf = np.array([qfx,qfy])

    img = cv.imread("./catkin_ws/src/a_star/worlds/circles.png")
    width = img.shape[0]
    height = img.shape[1]
    GRID_SIZE = 50

    ncols = int(width/GRID_SIZE)
    nrows = int(height/GRID_SIZE)
    grid = np.empty([nrows,ncols], Cell)
    
    WORLDSIZE = 80
    GRID_SIZE_R = WORLDSIZE/ncols

    while q0 is None:
        continue

    rng_x_img = range(0, width, GRID_SIZE)
    rng_y_img = range(0, height, GRID_SIZE)
    rng_x = [-WORLDSIZE/2+GRID_SIZE_R*x for x in range(0, ncols)]
    rng_y = [WORLDSIZE/2-GRID_SIZE_R*y for y in range(0, nrows)]
    ep = None
    sp = None
    for j in range(0,len(rng_y_img)):
        for i in range(0,len(rng_x_img)):
            cell_rect = img[rng_y_img[j]:rng_y_img[j]+GRID_SIZE,rng_x_img[i]:rng_x_img[i]+GRID_SIZE]
            if not (np.sum(cell_rect==0) > 0):
                grid[j,i] = Cell(j, i, 1)
            else:
                grid[j,i] = Cell(j, i, inf)
            #print("i:{} j:{}".format(i, j))
            #print("x_img:{} y_img:{}".format(rng_x_img[i], rng_y_img[j]))
            #print("x:{} y:{}".format(rng_x[i], rng_y[j]))
            #cv.imshow('img[{}][{}]'.format(i,j), cell_rect)
            #cv.waitKey(0)
            #cv.destroyAllWindows()
            if q0[0]>=rng_x[i] and q0[0]<=rng_x[i]+GRID_SIZE_R and \
                q0[1]<=rng_y[j] and q0[1]>=rng_y[j]-GRID_SIZE_R:
                if grid[j,i].value is inf:
                    raise RuntimeError('Start is not in a free cell')
                sp = grid[j,i]
            if qf[0]>=rng_x[i] and qf[0]<=rng_x[i]+GRID_SIZE_R and \
                qf[1]<=rng_y[j] and qf[1]>=rng_y[j]-GRID_SIZE_R:
                if grid[j,i].value is inf:
                    raise RuntimeError('Goal is not in a free cell')
                ep = grid[j,i]

    if ep is None:
        raise RuntimeError('Goal cell not found')
    if sp is None:
        raise RuntimeError('Start cell not found')

    fronteir = PriorityQueue()
    fronteir.put(Prioritize(0,sp))
    cost_so_far = dict()
    cost_so_far[sp] = 0
    came_from = dict()
    came_from[sp] = None

    while not fronteir.empty():
        cur_cell = fronteir.get().item
        
        if cur_cell.row == ep.row and cur_cell.col == ep.col:
            break

        for next in cur_cell.neighbors(grid):
            new_cost = cost_so_far[cur_cell] + next.value
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + abs(ep.row - next.row) + abs(ep.col - next.col)
                fronteir.put(Prioritize(priority,next))
                came_from[next] = cur_cell

    backtrace = []
    backtrace.append(ep)
    c_cell = ep
    while came_from[c_cell] is not None:
        c_cell = came_from[c_cell]
        backtrace.append(c_cell)

    # with open('data_wf_grid_25_13.csv','w', newline='') as csvfile:
    #     fieldnames = ['row','col','x','y','size','isStart','isGoal','isFree','value']
    #     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    #     writer.writeheader()
    #     for j in range(0,grid.shape[0]):
    #         for i in range(0,grid.shape[1]):
    #             writer.writerow({'row':j,'col':i,'x':grid[j,i].x,'y':grid[j,i].y,\
    #                 'size':GRID_SIZE_R,'isStart':grid[j,i].isStart,'isGoal':grid[j,i].isGoal,\
    #                 'isFree':grid[j,i].isFree,'value': grid[j,i].value})

    ep = 0.005
    alp = 0.5
    
    # with open('data_wf_25_13.csv','w', newline='') as csvfile:
    #     fieldnames = ['time','x','y','theta','Vx','Vy','Vlin','Vang',\
    #         'grad_atr_pot_x','grad_atr_pot_y','targ_cell_row','targ_cell_col']
    #     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    #     writer.writeheader()
    cur_target = backtrace.pop()
    cur_qf = np.array([rng_x[cur_target.col]+GRID_SIZE_R/2, rng_y[cur_target.row]-GRID_SIZE_R/2])
    while not rospy.is_shutdown() and np.linalg.norm(q0-qf) > ep:
        if q0 is not None and qf is not None and theta is not None:
            if len(backtrace) > 0 and np.linalg.norm(q0-cur_qf) < 0.1:
                cur_target = backtrace.pop()
                cur_qf = np.array([rng_x[cur_target.col]+GRID_SIZE_R/2, rng_y[cur_target.row]-GRID_SIZE_R/2])
                        
            d_atr_pot = attraction_potential(cur_qf)
            d_pot = d_atr_pot
            V = - alp*(d_pot)
                
            # Omnidirectional robot
            #vel_msg.linear.x = V[0]
            #vel_msg.linear.y = V[1]
            
            # Diff robot
            vel_msg.linear.x = cos(theta)*V[0]+sin(theta)*V[1]
            vel_msg.angular.z = (-sin(theta)*V[0]+cos(theta)*V[1])/d

            # writer.writerow({'time':rospy.Time.now(),'x':q0[0],'y':q0[1],\
            #     'theta':theta,'Vx':V[0],'Vy':V[1],'Vlin':vel_msg.linear.x,\
            #     'Vang':vel_msg.angular.z,'grad_atr_pot_x':d_atr_pot[0],\
            #     'grad_atr_pot_y':d_atr_pot[1],'targ_cell_row':idx[0],'targ_cell_col':idx[1]})

            rate.sleep()
            pub.publish(vel_msg)
    
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass