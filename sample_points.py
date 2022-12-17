#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 19 12:12:00 2020

@author: jelmer
"""

import numpy as np
import random
import matplotlib.pyplot as plt

def sample_points(width, height, pixels, D):
    
    coordinates = []

    N_x = int(width / D)                                                        #Number of samples in x-direction
    N_y = int(height / D)                                                       #Number of samples in y-direction
    N_S = 0                                                                     #Counter to count points that are really sampled
    
    i = 0
    j = 0
    
    while i < N_x:                                                              #For all x
        N_try = 0                                                               #Reset number of try's used
        while j < N_y:                                                          #For all y
            x = random.randint(int(i*width/N_x), int((i+1)*width/N_x - 1))      #Random x-location in neigbhour of grid point
            y = random.randint(int(j*height/N_y), int((j+1)*height/N_y - 1))    #Random y-location in neigbhour of grid point
            
            if int(pixels[y][x]) == 1:                                          #Check wether point is in free space
                coordinates.append([x,y])                                       #Append coordinate to list
                #plt.plot(x, y, marker='o', markersize=3, color='yellow')        #Plot the point
                N_S +=1                                                         #Increase number of sampled points
                j+=1                                                            #Increase y-index
                N_try = 0                                                       #Reset number of try's used
            elif N_try > 100:                                                   #Check wether algorithm has tried to often around same location
                j+=1                                                            #Increase y-index
                N_try = 0                                                       #Reset number of try's used
            else:
                N_try +=1                                                       #Increase number of try's used
                
        j = 0                                                                   #Reset y-index
        i+=1                                                                    #Increase x-index
        
    coordinates = np.asarray(coordinates)
        
    return coordinates, N_S