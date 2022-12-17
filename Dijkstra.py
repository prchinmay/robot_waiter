#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 15 15:23:38 2020

@author: jelmer
"""

import numpy as np

ed = np.linalg.norm                                                             #Define euclidean distance function

def Dijkstra(cor, con, start, end):
    
    #%% Set up initializes:
    
    i = start                                                                   #start node
    unexplored = []                                                             #list to save unexplored nodes
    explored = [start]                                                          #list to save explored nodes
    
    for k in con[start]:                                                        #Add neighbour nodes of start node to explored list.
        unexplored.append([k, start, ed(cor[i] - cor[k])])                      #template = [node, came from, weight]
        
    current_weight = 0                                                          #create variable to store weight
    
    itterations = [np.asarray(unexplored)]                                      #list to store all itteration steps
    
    #%% Itteration loop:
    
    while end != i:                                                             #Keep going until end node is picked by algorithm.
                    
        index = np.argmin(np.array(unexplored)[:,2])                            #Get index of node with least weight.
        next_node = unexplored[index][0]                                        #Set next_node to the node with least weigth.
        
        explored.append(next_node)                                              #Add the new_node to the list of explored nodes.
        #print("E: ", explored)
        
        for k in con[next_node]:                                                #For all the neigbhour nodes of the new node,
                if k not in explored:                                           #which are not already explored,
                    if k not in np.array(unexplored)[:,0].astype(int):          #and which are not already in the unexplored nodes list,
                        unexplored.append([k, next_node, current_weight + ed(cor[next_node] - cor[k])])         #add the node to the unexplored node list.
                    
                    else:                                                                                       #If the node is already in the unexplored node list,
                        ind = np.argwhere(np.array(unexplored)[:,0].astype(int) == k).flatten()[0]              #get the index of the node,
                        if current_weight + ed(cor[next_node] - cor[k]) < unexplored[ind][2]:                   #and check if the weight would be lower.
                            unexplored[ind] = [k, next_node, current_weight + ed(cor[next_node] - cor[k])]      #If so, update the node in the unexplored node list.
                        
        current_weight = unexplored[index][2]                                   #Update the current weight for the new node
        unexplored.pop(index)                                                   #Remove the new node from the unexplored nodes list
        
        if next_node != end:                                                    #If the new node is not the end node
            itterations.append(np.asarray(unexplored))                          #Add the itteration step to the list
            
        i = next_node                                                           #Set i to the new node
    
    #%% Getting route from the made itterations:
    
    route = [i]                                                                 #Create route list with end node as first.
    while i != start:                                                           #Keep going until algorithm has reached begin node.
        if i in itterations[-1][:, 0].astype(int):                              #If last node can be found in last itteration step,
            index = np.where(itterations[-1][:, 0].astype(int) == i)            #get the index of the last node,
            i = int(itterations[-1][index, 1])                                  #set i to the node before the last node,
            route.append(i)                                                     #and add this 'before node' to the route list.
            del itterations[-1]                                                 #Remove the last itteration step.
        
        else:                                                                   #If last node cannot be found in last itteration step,
            del itterations[-1]                                                 #remove the last itteration step.
    
    route = list(reversed(route))                                               #Reverse the order of the route (to make it from start to end)
    
    return route
            
        