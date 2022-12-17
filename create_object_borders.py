#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 19 11:07:10 2020

@author: jelmer
"""

from PIL import Image
import numpy as np
from scipy import signal
from colour_picking_check import point_in_free_space

def create_object_borders(floor_plan, border_size):
    
    #%% Create pixel array:
    width, height = floor_plan.size
    pixels = np.zeros([height, width])
    
    #%% Set pixels in free space to 1:
    for i in range(height):
        for j in range(width):
            if point_in_free_space(floor_plan, [j,i]):
                pixels[i][j] = 1
                
    #%% Create borders:
    pixels_upd = pixels
    
    for k in range(border_size):                                                #Itterate over border_size.
        counts = signal.convolve2d(pixels,np.ones((3,3)), mode='same')          #Calculate sum of pixels around current pixel.
        for i in range(height):
            for j in range(width):
                if pixels[i][j] == 1 and counts[i][j] != 9:                     #If current pixel is white and not all pixels around are white,
                    pixels_upd[i][j] = 0.5                                      #set current pixels to gray.
        pixels = pixels_upd
                
    #%% Convert pixels back to image and return:
    pixels = 255 * pixels_upd
    image = Image.fromarray(pixels)
                
    return pixels_upd, image