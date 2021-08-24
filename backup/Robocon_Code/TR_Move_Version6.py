"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import os

import numpy as np

import math

import matplotlib.pyplot as plt

import pygame

show_animation = True

import tkinter as tk

import tkinter.messagebox as msb

from functools import partial

import moveroot as mr
import Object_Distance_Height as obj

class AStarPlanner:


    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                # print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))


        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)


        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
   

sx = 60
sy = 10

def program(gx, gy):
    global sx
    global sy

    # set obstacle positions
    ox, oy = [], []

    # External Barriers
    for i in range(0, 120):
        ox.append(i)
        oy.append(0.0)
    for i in range(0, 120):
        ox.append(120.0)
        oy.append(i)
    for i in range(0, 120):
        ox.append(i)
        oy.append(120.0)
    for i in range(0, 120):
        ox.append(0.0)
        oy.append(i)
    # Inside Barrier
    for i in range(0, 40):
        ox.append(20+i)
        oy.append(20.0)
    for i in range(0,40):
        ox.append(60+i)
        oy.append(20)
    for i in range(0, 40):
        ox.append(20+i)
        oy.append(100.0)
    for i in range(0, 40):
        ox.append(60+i)
        oy.append(100)
    for i in range(0, 40):
        ox.append(20)
        oy.append(60+i)
    for i in range(0, 40):
        ox.append(100)
        oy.append(60+i)
    for i in range(0, 40):
        ox.append(20)
        oy.append(20+i)
    for i in range(0, 40):
        ox.append(100)
        oy.append(20+i)

    # for i in range(0, 20):
    #     ox.append(40-i)
    #     oy.append(i+20)
    # for i in range(0, 20):
    #     ox.append(i+80)
    #     oy.append(i+20)
    # for i in range(0, 20):
    #     ox.append(i+20)
    #     oy.append(i+80)
    # for i in range(0, 20):
    #     ox.append(100-i)
    #     oy.append(i+80)

    #Half Y
    for i in range(0, 20):
        ox.append(i)
        oy.append(60)
    for i in range(0, 20):
        ox.append(100+i)
        oy.append(60)

    #Half X
    for i in range(0, 80):
        ox.append(60)
        oy.append(20+i)

    # Arrow Shelter
    for i in range(0,10):
        ox.append(i)
        oy.append(5)
    for i in range(0,5):
        ox.append(10)
        oy.append(i)


    grid_size = 5.0  # [m]
    robot_radius = 4.5  # [m]
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)
    rx.reverse()
    ry.reverse()

    i = 0
    up = 0
    down = 0
    rightdown  = 0 
    rightup  = 0
    right  = 0
    left = 0
    leftup = 0
    leftdown = 0
    


    for i in range(len(rx)-1):
        if((rx[i] == rx[i+1]) | (ry[i] == ry[i+1])):
            if(ry[i] == ry[i+1]):
                if(rx[i] > rx[i+1]):
                    startx = rx[i] + (5*left)
                    left = left+1
                elif(rx[i] < rx[i+1]):
                    startx = rx[i] - (5*right)
                    right = right+1


                if (i == (len(rx)-2)):
                    if right>0:
                        mr.cubic_Polynominal(startx, rx[i+1], ry[i], ry[i+1], right, 0, 0, 5, 0, 0)
                        right = 0
                        continue
                    
                    elif left>0:
                        mr.cubic_Polynominal(startx, rx[i+1], ry[i], ry[i+1], left, 0, 0, 5, 0, 0)
                        left = 0
                        continue 

                elif(ry[i+1] != ry[i+2]):
                    if right>0:
                        mr.cubic_Polynominal(startx, rx[i+1], ry[i], ry[i+1], right, 0, 0, 5, 0, 0)
                        right = 0
                        continue
                    
                    elif left>0:
                        mr.cubic_Polynominal(startx, rx[i+1], ry[i], ry[i+1], left, 0, 0, 5, 0, 0)
                        left = 0
                        continue    
       
            #move up and down
            elif(rx[i] == rx[i+1]):
                if(ry[i] > ry[i+1]):
                    starty = ry[i] + (5*down)
                    down = down+1

                elif(ry[i] < ry[i+1]):
                    starty = ry[i] - (5*up)
                    up = up+1

                    
                if (i == (len(rx)-2)):
                    if up>0:
                        mr.cubic_Polynominal(rx[i], rx[i+1], starty, ry[i+1], up, 0, 0, 0, 0, 0)
                        up = 0
                    elif down>0:
                        mr.cubic_Polynominal(rx[i], rx[i+1], starty, ry[i+1], down, 0, 0, 0, 0, 0)
                        down = 0
                elif(rx[i+1] != rx[i+2]):
                    if up>0:
                        mr.cubic_Polynominal(rx[i], rx[i+1], starty, ry[i+1], up, 0, 0, 0, 0, 0)
                        up = 0
                    elif down>0:
                        mr.cubic_Polynominal(rx[i], rx[i+1], starty, ry[i+1], down, 0, 0, 0, 0, 0)
                        down = 0
           
        else:  
            #move leftup and leftdown
            if(rx[i] > rx[i+1]):
                # print(" leftdown or leftup")
                if(ry[i] > ry[i+1]):
                    starty = ry[i] + (5*leftdown)
                    startx = rx[i] + (5*leftdown)
                    leftdown= leftdown+1

                elif(ry[i] < ry[i+1]):
                    starty = ry[i] - (5*leftup)
                    startx = rx[i] + (5*leftup)
                    leftup = leftup+1
                    

                if (i == (len(rx)-2)):
                    if leftdown>0:
                        mr.cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], leftdown, 0, 0, 0, 0, 0)   
                        leftdown=0
                    elif leftup>0:
                        mr.cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], leftup, 0, 0, 0, 0, 0)    
                        leftup = 0
                elif((rx[i+1] == rx[i+2]) | (ry[i+1] == ry[i+2])):  
                    if leftdown>0:
                        mr.cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], leftdown, 0, 0, 0, 0, 0)                        
                        leftdown=0
                    elif leftup>0:
                        mr.cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], leftup, 0, 0, 0, 0, 0)                        
                        leftup = 0
                        
            #move rightup and rightdown
            elif(rx[i] < rx[i+1]):
                # print("rightdown or rightup")
                if(ry[i] > ry[i+1]):
                    starty = ry[i] + (5*rightdown)
                    startx = rx[i] - (5*rightdown)
                    rightdown = rightdown + 1

                elif(ry[i] < ry[i+1]):
                    starty = ry[i] - (5*rightup)
                    startx = rx[i] - (5*rightup)
                    rightup = rightup+1


                if(i == (len(rx)-2)):
                    if rightdown>0:
                        mr.cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], rightdown, 0, 0, 0, 0, 0)                        
                        rightdown =0
                        continue
                    elif rightup>0:
                        mr.cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], rightup, 0, 0, 0, 0, 0)                        
                        rightup=0
                        continue
                
                elif((ry[i+1] == ry[i+2]) | (rx[i+1] == rx[i+2]) ):
                    if rightdown>0:
                        mr.cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], rightdown, 0, 0, 0, 0, 0)  
                        rightdown =0
                        continue
                    elif rightup>0:
                        mr.cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], rightup, 0, 0, 0, 0, 0)          
                        rightup=0
                        continue
  #set current location
    sx = gx
    sy = gy

    # if show_animation:  # pragma: no cover
    #     plt.plot(rx, ry, "-r")
    #     plt.pause(0.001)
    #     plt.show()

def control():

    print("Please input the value: ")
    id = int(input())
    if(id==1):
        program(110,50)
        return 110, 50
    elif(id==2):
        program(10,50)
        return 10, 50
    elif(id==3):
        program(115,5)
        return 115,5
    elif(id==4):
        program(110,50)
        return 110,50
    elif(id==5):
        program(10,50)
        return 10,50
    elif(id==6):
        program(110,30)
        return 110,30

       


