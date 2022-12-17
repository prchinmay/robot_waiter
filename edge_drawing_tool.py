#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec  9 08:52:41 2020

@author: jelmer
"""

import matplotlib.pyplot as plt

def draw_edge(node1, node2, color='blue', line="-"):
    plt.plot(node1[0], node1[1], marker='o', markersize=3, color='green')       #Draw marker for first node
    plt.plot(node2[0], node2[1], marker='o', markersize=3, color='green')       #Draw marker for second node
    plt.plot([node1[0], node2[0]], [node1[1], node2[1]], line, color=color)          #Draw edge between nodes
    