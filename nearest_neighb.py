# -*- coding: utf-8 -*-
"""
Created on Fri Dec 11 01:19:28 2020

@author: prchi
"""

import random
import math
import numpy as np
from scipy import spatial
from scipy.spatial import cKDTree

def near_neighb(sample_x,sample_y,DIST):
    points = np.c_[sample_x,sample_y]
    tree = spatial.cKDTree(points)
    ind=[]
    
    for i in range(len(points)):
        indices=tree.query_ball_point(points[i], DIST)
        ind.append(indices)
        
    return np.array(ind, dtype=object)