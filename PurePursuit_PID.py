# -*- coding: utf-8 -*-
"""
Created on Thu Dec 31 15:03:50 2020

@author: prchi
"""

#https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py

import numpy as np
import math

#%% Robot physical parameters:
WB = 30 * 100/70  # [pixels] wheel base of vehicle
a = WB/2 # [inches] Distance to front axle cg
b = WB-a # [inches] Distance of rear axle from cg
m = 10 #[Kg] Vehicle mass
I = (1/12)*m*WB**2 #[kg*inch^2] Moment of Intertia

#%% Robot limits:
max_steering_angle = 1.5 #[radians]
max_acceleration = 100 * 100/70 #[cm/s^2]
target_speed = 50 * 100/70  # [cm/s]

#%% Controller gains:
k = 0.1  # look forward gain
Lfc = WB*0.5  # [pixels] look-ahead distance
Kp = 1  # speed proportional gain
Ki = 0.01
Kd = 0.0001
dt = 0.1  # [s] time tick
#%% State class:
class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, dx=0.0, dy=0.0, dyaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.dx = dx
        self.dy = dy                                                            #State variables
        self.dyaw = dyaw                                                        
        self.v = np.sqrt(self.dx**2 + self.dy**2)
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        
    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y                                              #Calculate Rear axle position
        return math.hypot(dx, dy)
    
    def update_vel(self, x, y, yaw, dx, dy, dyaw, delta):
        # velocity projection so that velocities match with constraints
        # from Advanced Dynamics, Valerie & Schwab, 3rd edition, page 414
        phi = self.yaw
        q = np.array([x,y,yaw])
        dq = np.array([dx, dy, dyaw])
        b = WB-a
        
        C = np.array([[    np.sin(phi),         -np.cos(phi),            a],
                      [np.sin(phi+delta), -np.cos(phi+delta), -b*np.cos(delta)]]) #projection of velocities into Nonholonomic space

        CCTCC = np.dot(np.linalg.pinv(C),C)
        dq += -np.dot(CCTCC,dq)

        self.x, self.y, self.yaw = q[0], q[1], q[2]
        self.dx, self.dy, self.dyaw = dq[0], dq[1], dq[2]
         
    
    # forward dynamics
    def update_dyn(self,delta,Fa):
        q = np.array([self.x, self.y, self.yaw])
        dq = np.array([self.dx, self.dy, self.dyaw])
        
        cphi = np.cos(q[2])
        sphi = np.sin(q[2])
        cdeltaphi = np.cos(q[2]+delta)
        sdeltaphi = np.sin(q[2]+delta)                                          #Dynamic equations
        cdelta    = np.cos(delta)
        dx   = dq[0]
        dy   = dq[1]
        dphi = dq[2]

        
        rhs = np.array([Fa*cphi, Fa*sphi, 0, -cphi*dx*dphi-sphi*dy*dphi, -cdeltaphi*dx*dphi-sdeltaphi*dy*dphi])

        # inertia matrix
        M = np.array([[    m,          0,         0,  sphi,  sdeltaphi],
                      [         0,     m,         0, -cphi, -cdeltaphi],
                      [         0,          0,    I,     a,  -b*cdelta],
                      [      sphi,      -cphi,         a,     0,      0],
                      [ sdeltaphi, -cdeltaphi, -b*cdelta,     0,      0]])

        # accelerations
        sol = np.linalg.solve(M, rhs)      
        ddq = sol[0:3]  #only return the accelerations                          # solve command returs accelerations
        
        self.dx += ddq[0]*dt
        self.dy += ddq[1]*dt
        self.dyaw += ddq[2]*dt
        self.v = np.sqrt(self.dx**2 + self.dy**2)                               #Updating states by integration
        
        self.x += self.dx*dt
        self.y += self.dy*dt
        self.yaw += self.dyaw*dt
        
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))


#%% States class:
class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.dx = []
        self.dy = []                                                            #Initializing arrays for storing states 
        self.dyaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.dx.append(state.dx)                                                #Function for appending
        self.dy.append(state.dy)
        self.dyaw.append(state.dyaw)
        self.v.append(state.v)
        self.t.append(t)


#%% PID controller:
def proportional_control(target_speed, current_speed):
    acc = Kp * target_speed- current_speed                                      # Target velocity tracking using PID
    acc_max = max_acceleration
    
    if acc>acc_max:
        acc = acc_max
    if acc<-acc_max:                                                            # Limit on maximum acceleration
        acc = -acc_max
        
    Fa = m*acc                                                                  # Return Propulsive force to be fed
    return Fa                                                                   # as input to dynamic equations

#%% Class TaretCourse:

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])     #Function to search target point on 
                if distance_this_index < distance_next_index:                   #the spline path to be followed
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind
            
   
        Lf = (k * state.v) + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf
    

#%% Pure Pursuit Controller:
#https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw        #Calculate difference in heading direction and target direction
                                                                                
    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)                    # Assign this difference as delta(steering input)
    
    delta_max = max_steering_angle  
    if delta>delta_max:
        delta = delta_max                                                       # Limit on max steering
    if delta<-delta_max:
        delta = -delta_max

    return delta, ind


#%% Main function:
def control(cx, cy, init_pose):

    delta = []
    f_prop = []
    state = State(x=cx[0], y = cy[0], yaw = init_pose[2], dx=0.0, dy=0.0, dyaw=0.0)  # initial state

    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)
    se = 0
    pe= 0

    while lastIndex > target_ind:
        er = target_speed - state.v
        se = se + (er*dt)
        Der = (er-pe)/dt
        Fa = proportional_control(target_speed, state.v) + Ki*se + Kd*Der        # PID control
        
                              
        di, target_ind = pure_pursuit_steer_control(                            # Pure pursuit control
            state, target_course, target_ind)
        
        
        state.update_vel(state.x, state.y, state.yaw, state.dx, state.dy, state.dyaw, di) 
        state.update_dyn(di, Fa)                                                #Update states from dynamic equations

        time += dt
        states.append(time, state)
        delta.append(di)
        f_prop.append(Fa)
        pe = er

    states.x.append(cx[len(cx)-1])
    states.y.append(cy[len(cx)-1])

    return np.array(states.x), np.array(states.y), np.array(states.yaw), np.array(delta), np.array(states.v), np.array(f_prop)



