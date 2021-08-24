"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""
import serial, time

import os

import numpy as np

import math

import matplotlib.pyplot as plt

import pygame

show_animation = True

import tkinter as tk

import tkinter.messagebox as msb

from functools import partial

# Motor control
start_point = 0
arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
def trans(replysize, data=bytearray()):
    respon = [0] * replysize
    arduino.write(data)
    while (arduino.in_waiting == 0):
        # print("wait")
        continue
    while (arduino.inWaiting() > 0):
        arduino.readinto(respon)
    for j in range(0, replysize):
        hex(respon[j])


def stopmotor(idm):
    mstop = bytearray(5)
    mstop[0] = 0x3e
    mstop[1] = 0x81
    mstop[2] = idm
    mstop[3] = 0
    mstop[4] = (mstop[0] + mstop[1] + mstop[2] + mstop[3]) & 0x00ff
    trans(5, mstop)


def opmotor(idm):
    mop = bytearray(5)
    mop[0] = 0x3e
    mop[1] = 0x88
    mop[2] = idm

    mop[3] = 0x00
    mop[4] = (mop[0] + mop[1] + mop[2] + mop[3]) & 0x00ff
    trans(5, mop)


def runmotorspeed(idm, speed):
    mtspeed = speed * 100

    mrunspd = bytearray(10)
    mrunspd[0] = 0x3e
    mrunspd[1] = 0xa2
    mrunspd[2] = idm
    mrunspd[3] = 0x04
    mrunspd[4] = (mrunspd[0] + mrunspd[1] + mrunspd[2] + mrunspd[3]) & 0x00ff
    for i in range(5, 9):
        mrunspd[i] = (mtspeed >> 8 * (i - 5)) & 0x000000ff
    mrunspd[9] = (mrunspd[5] + mrunspd[6] + mrunspd[7] + mrunspd[8]) & 0x00ff
    trans(13, mrunspd)
    


def movemotor(x, y, d):
    R = 3.5
    r = 0.76
    idm1 = 1  # motor id
    idm2 = 2  # motor id
    idm3 = 3  # motor id

    # speed motor
    motor1 = (((-math.sin(math.radians(60)) * math.cos(math.radians(60))) * x) + (
                (math.cos(math.radians(60)) * math.cos(math.radians(60))) * y) + (R * d)) / r
    motor1 = int(motor1 * 100)
    # print(" motor1 = ", motor1)

    motor2 = (((-math.sin(math.radians(180)) * math.cos(math.radians(60))) * x) + (
                (math.cos(math.radians(180)) * math.cos(math.radians(60))) * y) + (R * d)) / r
    motor2 = int(motor2 * 100)
    # print(" motor2 = ", motor2)

    motor3 = (((-math.sin(math.radians(300)) * math.cos(math.radians(60))) * x) + (
                (math.cos(math.radians(300)) * math.cos(math.radians(60))) * y) + (R * d)) / r
    motor3 = int(motor3 * 100)
    # print(" motor3 = ", motor3)

    runmotorspeed(idm1, motor1)
    runmotorspeed(idm2, motor3)
    runmotorspeed(idm3, motor2)

def cubic_Polynominal(x0, x1, y0, y1, tf):
    a1x = 0
    a1y = 0
    a2x = (3 * (x1 - x0)) / (tf * tf)
    a2y = (3 * (y1 - y0)) / (tf * tf)

    a3x = (-2 / (tf * tf * tf)) * (x1 - x0)
    a3y = (-2 / (tf * tf * tf)) * (y1 - y0)

    t = 0
    tf1 = int(tf * 100)
    for t in range(tf1+1):
        time1 = time.perf_counter()
        vx = a1x + (2 * a2x * (t / 100)) + (3 * a3x * (t / 100) * (t / 100))
        vy = a1y + (2 * a2y * (t / 100)) + (3 * a3y * (t / 100) * (t / 100))
        time2 = time.perf_counter()
        movemotor(vx, vy, 0);
        time.sleep(0.001 - (time2 - time1))

## End Motor Control
        
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
        # print("min_x:", self.min_x)
        # print("min_y:", self.min_y)
        # print("max_x:", self.max_x)
        # print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        # print("x_width:", self.x_width)
        # print("y_width:", self.y_width)

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
   

sx = 115
sy = 5

def program(gx, gy):
    #arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    #time.sleep(1)
    # print(__file__ + " start!!")
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
        ox.append(40+i)
        oy.append(20.0)
    for i in range(0, 40):
        ox.append(40+i)
        oy.append(100.0)
    for i in range(0, 40):
        ox.append(20)
        oy.append(40+i)
    for i in range(0, 40):
        ox.append(100)
        oy.append(40+i)

    for i in range(0, 20):
        ox.append(40-i)
        oy.append(i+20)
    for i in range(0, 20):
        ox.append(i+80)
        oy.append(i+20)
    for i in range(0, 20):
        ox.append(i+20)
        oy.append(i+80)
    for i in range(0, 20):
        ox.append(100-i)
        oy.append(i+80)

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

    # start and goal position
    # sx = 10.0  # [m]
    # sy = 10.0  # [m]
    # gx = 50.0  # [m]
    # gy = 50.0  # [m]
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
        # print("i= ", i)
        if((rx[i] == rx[i+1]) | (ry[i] == ry[i+1])):
            # print("x== or y==")
             #move left and right
            if(ry[i] == ry[i+1]):
                # print("y==")
                if(rx[i] > rx[i+1]):
                    startx = rx[i] + (5*left)
                    left = left+1
                elif(rx[i] < rx[i+1]):
                    startx = rx[i] - (5*right)
                    right = right+1


                    # print("move straight right"+str(right))

                if (i == (len(rx)-2)):
                    # print("Start Move Motor")
                    if right>0:
                        # runmotor(speed,0,0)
                        #time.sleep(right)
                        # print("Motor is move right" + str(right))
                        cubic_Polynominal(startx, rx[i+1], ry[i], ry[i+1], right)
                        right = 0
                        # startx = 0
                        continue
                    
                    elif left>0:
                        # runmotor(-speed,0,0)
                        #time.sleep(left)
                        # print("Motor is move left " + str(left))
                        cubic_Polynominal(startx, rx[i+1], ry[i], ry[i+1], left)
                        left = 0
                        # startx = 0
                        continue 

                elif(ry[i+1] != ry[i+2]):
                    #print("Start Move Motor")
                    if right>0:
                        # runmotor(speed,0,0)
                        #time.sleep(right)
                        # print("Motor is move right" + str(right))
                        cubic_Polynominal(startx, rx[i+1], ry[i], ry[i+1], right)
                        right = 0
                        # startx = 0
                        continue
                    
                    elif left>0:
                        # runmotor(-speed,0,0)
                        #time.sleep(left)
                        # print("Motor is move left " + str(left))
                        cubic_Polynominal(startx, rx[i+1], ry[i], ry[i+1], left)
                        left = 0
                        # startx = 0
                        continue    
                #else:
                #    print("b ouy")
            #move up and down
            elif(rx[i] == rx[i+1]):
                # print("x==")
                if(ry[i] > ry[i+1]):
                    starty = ry[i] + (5*down)
                    down = down+1

                elif(ry[i] < ry[i+1]):
                    starty = ry[i] - (5*up)
                    up = up+1

                    
                if (i == (len(rx)-2)):
                    if up>0:
                        # runmotor(0,speed,0)
                        #time.sleep(up)
                        # print("Motor is move up" + str(up))
                        cubic_Polynominal(rx[i], rx[i+1], starty, ry[i+1], up)
                        up = 0
                        # starty = 0
                    elif down>0:
                        # runmotor(0,-speed,0)
                        #time.sleep(down)
                        # print("Motor is move down" + str(down))
                        cubic_Polynominal(rx[i], rx[i+1], starty, ry[i+1], down)
                        down = 0
                        # starty = 0
                elif(rx[i+1] != rx[i+2]):
                    if up>0:
                        # runmotor(0,speed,0)
                        #time.sleep(up)
                        # print("Motor is move up" + str(up))
                        cubic_Polynominal(rx[i], rx[i+1], starty, ry[i+1], up)
                        up = 0
                        # starty = 0
                    elif down>0:
                        # runmotor(0,-speed,0)
                        #time.sleep(down)
                        # print("Motor is move down" + str(down))
                        cubic_Polynominal(rx[i], rx[i+1], starty, ry[i+1], down)
                        down = 0
                        # starty = 0
           
        else:  
            #move leftup and leftdown
            # print("x# and y#")
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
                        # runmotor(-speed,-speed,0)
                        #time.sleep(leftdown)
                        # print("Motor is leftdown" + str(leftdown))
                        cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], leftdown)                        
                        # startx = 0
                        # starty = 0
                        leftdown=0
                    elif leftup>0:
                        # runmotor(-speed,speed,0)
                        #time.sleep(leftup)
                        # print("Motor is leftup" + str(leftup))
                        # print ("starty = ",starty)
                        # print ("startx = ",startx)
                        # print("rx = ", rx[i+1])
                        # print("ry = ", ry[i+1])
                        cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], leftup)    
                                            
                        # startx = 0
                        # starty = 0
                        leftup = 0
                elif((rx[i+1] == rx[i+2]) | (ry[i+1] == ry[i+2])):  
                    if leftdown>0:
                        # runmotor(-speed,-speed,0)
                        #time.sleep(leftdown)
                        # print("Motor is leftdown" + str(leftdown))
                        cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], leftdown)                        
                        # startx = 0
                        # starty = 0
                        leftdown=0
                    elif leftup>0:
                        # runmotor(-speed,speed,0)
                        #time.sleep(leftup)
                        # print("Motor is leftup" + str(leftup))
                        # print ("starty = ",starty)
                        # print ("startx = ",startx)
                        # print("rx = ", rx[i+1])
                        # print("ry = ", ry[i+1])
                        cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], leftup)                        
                        # startx = 0
                        # starty = 0
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
                        # runmotor(speed,-speed,0)
                        #time.sleep(rightdown)
                        # print("Motor is rightdown"+ str(rightdown))
                        cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], rightdown)                        
                        # startx = 0
                        # starty = 0
                        rightdown =0
                        continue
                    elif rightup>0:
                        # runmotor(speed,speed,0)
                        #time.sleep(rightup)
                        # print("Motor is rightup"+ str(rightup))
                        cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], rightup)                        
                        # startx = 0
                        # starty = 0
                        rightup=0
                        continue
                
                elif((ry[i+1] == ry[i+2]) | (rx[i+1] == rx[i+2]) ):
                    if rightdown>0:
                        # runmotor(speed,-speed,0)
                        #time.sleep(rightdown)
                        # print("Motor is rightdown"+ str(rightdown))
                        cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], rightdown)                        
                        # startx = 0
                        # starty = 0
                        rightdown =0
                        continue
                    elif rightup>0:
                        # runmotor(speed,speed,0)
                        #time.sleep(rightup)
                        # print("Motor is rightup"+ str(rightup))
                        cubic_Polynominal(startx, rx[i+1], starty, ry[i+1], rightup)                        
                        # startx = 0
                        # starty = 0
                        rightup=0
                        continue
    # print("rx =",rx)
    # print("ry =",ry)    
    sx = gx
    sy = gy
    # print(sx)

    # if show_animation:  # pragma: no cover
    #     plt.plot(rx, ry, "-r")
    #     plt.pause(0.001)
    #     plt.show()

def control():

    print("Please input the value: ")
    id = int(input())
    if(id==1):
        program(55,15)
        return 55, 15
    elif(id==2):
        program(5,10)
        return 5, 10
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

    


# if __name__ == '__main__':
#     program()
# top = tk.Tk()
# top.title("Robot Controller")
# top.geometry("475x350")
# G1 = tk.Button(top, width=15, height=1, text ="GetArrowDR", command=lambda: program(55,15))
# G1.place(x=25, y=25)
# G2 = tk.Button(top, width=15, height=1, text ="GetArrow", command=lambda: program(5,10))
# G2.place(x=175, y=25)
# R1 = tk.Button(top, width=15, height=1, text ="Restart", command = partial(program,115,5))
# R1.place(x=325,y=25)
# S1 = tk.Button(top, width=15, height=1, text ="ShotTypeI", command = partial(program,110,50))
# S1.place(x=25,y=75)
# S2 = tk.Button(top, width=15, height=1, text ="ShotTypeII", command = partial(program,10,50))
# S2.place(x=175,y=75)
# S1 = tk.Button(top, width=15, height=1, text ="ShotTypeIII", command = partial(program,110,30))
# S1.place(x=325,y=75)

# # set specific destination to go
# canvas1 = tk.Canvas(top)
# canvas1.place(x=25,y=125)
# canvas1.create_text(10,25,text="Ox")
# canvas1.create_text(160,25,text="Oy")
# entry1 = tk.Entry(top,width=13)
# canvas1.create_window(75, 25, window=entry1)
# entry2 = tk.Entry(top,width=13)
# canvas1.create_window(220,25, window=entry2)

# def gospec():
#     gx = int(entry1.get())
#     gy = int(entry2.get())
#     program(gx,gy)
# button1 = tk.Button(top,width=15, height=1, text='Move', command=gospec)
# button1.place(x=325,y=140)

# top.mainloop()
