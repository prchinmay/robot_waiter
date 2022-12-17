#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  8 11:37:58 2020

@author: jelmer
"""

import matplotlib.pyplot as plt

def point_in_free_space(floor_plan, point, plot=False):
    pixels = floor_plan.load()                                                  #Define pixels of image
    pixel_RGB = list(pixels[point[0], point[1]][0:3])                           #Get RGB of specefic pixel
    
    if plot == True:                                                            #Possibility to plot checked pixel
        plt.plot(point[0], point[1], marker='o', markersize=5, color='red')
        plt.pause(0.0001)
      
    return (pixel_RGB == [255, 255, 255])                                       #Return true if pixel is white (and thus in free space)