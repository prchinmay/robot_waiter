#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan  5 09:05:49 2021

@author: jelmer
"""

import pygame as pg

def plot_nodes(coords, screen):
    for i in coords:
        pg.draw.circle(screen, (255, 255, 0), (i[0], i[1]), 4)
        
def plot_connections(connections, coords, screen):
    for i in range(len(connections)):
        for j in connections[i]:
            pg.draw.line(screen, (0, 255, 225), (coords[i][0], coords[i][1]), (coords[j][0], coords[j][1]))
            
def plot_original_route(route, coords, screen):
    for i in range(len(route)):
        if i != 0:
            p1 = (coords[route[i-1]][0], coords[route[i-1]][1])
            p2 = (coords[route[i]][0], coords[route[i]][1])
            pg.draw.line(screen, (0, 0, 255), p1, p2, width=2)
            
def plot_smoothed_route(cx, cy, screen):
    for i in range(len(cx)):
        if i != 0:
            pg.draw.line(screen, (255, 0, 0), (cx[i-1], cy[i-1]), (cx[i], cy[i]), width=3)
            
def plot_travelled_route(t, tx, ty, screen):
    for i in range(t-1):
        if i != 0 and i < len(tx)-1:
            pg.draw.line(screen, (0, 0, 0), (tx[i-1], ty[i-1]), (tx[i], ty[i]), width=5)
            
def plot_privious_travelled_route(ptx, pty, screen):
    for i in range(len(ptx)):
        if i != 0:
            pg.draw.line(screen, (0, 0, 0), (ptx[i-1], pty[i-1]), (ptx[i], pty[i]), width=3)
            
def plot_base(base_init, yaw_deg, center, screen):
    base = pg.transform.rotate(base_init, -yaw_deg)
    base_rect = base.get_rect()
    
    base_rect.center = center
    screen.blit(base, base_rect)
    
def plot_rear_wheel(rear_wheel_init, yaw_deg, rear, screen):
    rear_wheel = pg.transform.rotate(rear_wheel_init, -yaw_deg)
    rear_wheel_rect = rear_wheel.get_rect()
    
    rear_wheel_rect.center = rear
    screen.blit(rear_wheel, rear_wheel_rect)
    
def plot_front_wheel(front_wheel_init, yaw_deg, steer_deg, front, screen):
    front_wheel = pg.transform.rotate(front_wheel_init, -yaw_deg-steer_deg)
    front_wheel_rect = front_wheel.get_rect()
    
    front_wheel_rect.center = front
    screen.blit(front_wheel, front_wheel_rect)