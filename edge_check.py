#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 12 13:26:09 2020

@author: jelmer
"""


import matplotlib.pyplot as plt
from edge_drawing_tool import draw_edge

def edges_in_free_space(NN, coordinates, pixels, plot_delay=False ,plot_search=False, debug=False):
    
    connections = []                                                            #list to store all connections
    
    for i in range(len(NN)):                                                    #For all nodes
        
        node_connections = []                                                   #list to store connections for one node
        
        for j in NN[i]:                                                         #For all neigbours of node
            if j != i:                                                          #Not for self
                in_free_space = True
                
                delta_x = coordinates[:,0][i] - coordinates[:,0][j]             #Determine x distance
                delta_y = coordinates[:,1][i] - coordinates[:,1][j]             #Determine y distance
                
                x_i, y_i = coordinates[:,0][i], coordinates[:,1][i]             #Set start point to coordinates of node i
                t = 0                                                           #counter, used for plot_search
                
                if abs(delta_x) > abs(delta_y):                                 #If x distance is largest:
                    while in_free_space and x_i != coordinates[:,0][j]:         #For all pixels between x[i] and x[j]
                        
                        if debug == True   :                                     #Debug option
                            print(x_i)
                            print(y_i, '\n')
                        
                        if int(pixels[int(y_i)][int(x_i)]) == 1 and int(pixels[int(y_i+1)][int(x_i)]) == 1:     #If pixel is in free space:
                            if plot_search == True and t%10 == 0:                                               #Plot every 10th checked pixel
                                plt.plot(x_i, y_i, marker='o', markersize=1, color='green')
                                plt.pause(0.001)
                                
                            t +=1
                            x_i += - delta_x / abs(delta_x)                                     #Incresease x_i
                            if delta_y != 0:
                                y_i += - delta_y / abs(delta_y) * abs(delta_y) / abs(delta_x)   #Incresease y_i
                                
                        else:
                            in_free_space = False                               #Stop if pixel is not in free space
                    
                else:                                                           #If y distance is largest:
                    while in_free_space and y_i != coordinates[:,1][j]:         #For all pixels between y[i] and y[j]
                        
                        if debug == True:                                        #Debug option
                            print(x_i)
                            print(y_i, '\n')
                        
                        if int(pixels[int(y_i)][int(x_i)]) == 1 and int(pixels[int(y_i)][int(x_i+1)]) == 1:      #If pixel is in free space:
                            if plot_search == True and t%10 == 0:                                                #Plot every 10th checked pixel
                                plt.plot(x_i, y_i, marker='o', markersize=1, color='green')
                                plt.pause(0.001)
                                
                            t +=1
                            y_i += - delta_y / abs(delta_y)                                     #Incresease y_i
                            if delta_x != 0:                                
                                x_i += - delta_x / abs(delta_x) * abs(delta_x) / abs(delta_y)   #Incresease x_i
                                
                        else:
                            in_free_space = False                               #Stop if pixel is not in free space
                    
                
                if in_free_space:                                               #If all checked pixels where in free space:
                    node_connections.append(j)                                  #Node is saved
                    
                    #draw_edge([coordinates[:,0][i],coordinates[:,1][i]], [coordinates[:,0][j],coordinates[:,1][j]], color="cyan")     #Egde is drawn
                    #if plot_delay == True:                                      #For animation style
                    #    plt.pause(0.00001)
                
        connections.append(node_connections)                                    #All connections of node are saved
        
    return connections
