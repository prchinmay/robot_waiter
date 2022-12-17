# -*- coding: utf-8 -*-
"""
Created on Tue Dec 22 02:44:17 2020

@author: prchi
"""

import numpy as np
from scipy.interpolate import splev, splrep



#https://www.math24.net/curvature-radius/#:~:text=The%20radius%20of%20curvature%20of,%E2%80%B2%E2%80%B2(x)%7C.

#%% Find bspline path:
#https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splprep.html
#https://docs.scipy.org/doc/scipy/reference/tutorial/interpolate.html

def bspline_path(x, y, sn, k):
    t = range(len(x))
    x_tup = splrep(t, x, k=k)                                                   #parametrize x,y on another variable t
    y_tup = splrep(t, y, k=k)

    
    x_list = list(x_tup)
    x_list[1] = x


    y_list = list(y_tup)
    y_list[1] = y

    points = np.linspace(0.0, len(x)-1, sn)                                     
    tx = splev(points , x_list)                                                 #Extrapolate to cartesian coordinates
    ty = splev(points , y_list)

    
    dx = splev(points , x_list, der=1)                                          #1st order derivates along x & y
    dy = splev(points , y_list, der=1)
    
    ddx = splev(points , x_list, der=2)
    ddy = splev(points , y_list, der=2)                                         #2nd order derivates along x & y
    
    return np.array(tx), np.array(ty), np.array(dx), np.array(dy), np.array(ddx), np.array(ddy)

#%% Find Radius of curvature at each point in the path:
#https://www.math24.net/curvature-radius/
def radius_of_curvature(dx, dy, ddx, ddy, n_edges):
    
    R = np.power((dx**2 + dy**2), 3/2)/np.absolute(dx*ddy - dy*ddx)             #Radius of curvature formula for parametric curve
    
    R_avg=[]
    for i in np.arange(0, n_edges, 1):
        avg = np.mean(R[i*30:(i+1)*30])                                         #Calculate average Radius of curvature for each edge
        R_avg.append(avg)
    return R

#%% Find violations from minimum turning radius of Robot in the given path:
def prob_pnts(route_x, route_y, R_avg):
    d_x=[]
    d_y=[]
    indx=[]
    for i in range(len(R_avg)-1):
        if R_avg[i]<100 and i!=0 and i!=len(R_avg)-1:                           #Check for those points whose Radius of curvature 
            d_x.append([route_x[i],route_x[i+1]])                               # is less than minumum
            d_y.append([route_y[i],route_y[i+1]])
            indx.append([i,i+1])
          
    d_x=np.array(d_x).ravel()
    d_y=np.array(d_y).ravel()
    indx=np.array(indx).ravel()
   
    return indx,d_x,d_y







