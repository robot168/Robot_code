#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 example             ##
#####################################################

# First import the library
import pyrealsense2 as rs
import time
import TR_Move_Version6 as tr
import v2_moverobot as mo
from threading import Thread
# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg) 
x = 0
y = 0
def tracking():
    try:
        while(True):
            # Wait for the next set of frames from the camera
            frames = pipe.wait_for_frames()

                # Fetch pose frame
            pose = frames.get_pose_frame()

            if pose:
                    # Print some of the pose data to the terminal
                data = pose.get_pose_data()
                # print("Frame #{}".format(pose.frame_number))
                # print("Position: {}".format(data.translation))
                global x, y
                x = data.translation.x
                y = data.translation.y 
                x = round(x, 4)
                y = round(y, 4)
                # print("X:", x)
                # print("Y:", y)
      
               
    
    finally:
        pipe.stop()   

# def motor_run():
#     global x, y
#     print ("X when motor is start {}".format(x))
#     print ("Y when motor is start {}".format(y))
#     # program()
#     tr.top = tr.tk.Tk()
#     tr.top.title("Robot Controller")
#     tr.top.geometry("475x350")
#     tr.G1 = tr.tk.Button(tr.top, width=15, height=1, text ="GetArrowDR", command=lambda: tr.program(55,15))
#     tr.G1.place(x=25, y=25)
#     tr.G2 = tr.tk.Button(tr.top, width=15, height=1, text ="GetArrow", command=lambda: tr.program(5,10))
#     tr.G2.place(x=175, y=25)
#     tr.R1 = tr.tk.Button(tr.top, width=15, height=1, text ="Restart", command = tr.partial(tr.program,115,5))
#     tr.R1.place(x=325,y=25)
#     tr.S1 = tr.tk.Button(tr.top, width=15, height=1, text ="ShotTypeI", command = tr.partial(tr.program,110,50))
#     tr.S1.place(x=25,y=75)
#     tr.S2 = tr.tk.Button(tr.top, width=15, height=1, text ="ShotTypeII", command = tr.partial(tr.program,10,50))
#     tr.S2.place(x=175,y=75)
#     tr.S1 = tr.tk.Button(tr.top, width=15, height=1, text ="ShotTypeIII", command = tr.partial(tr.program,110,30))
#     tr.S1.place(x=325,y=75)


#     canvas1 = tr.tk.Canvas(tr.top)
#     canvas1.place(x=25,y=125)
#     canvas1.create_text(10,25,text="Ox")
#     canvas1.create_text(160,25,text="Oy")
#     tr.entry1 = tr.tk.Entry(tr.top,width=13)
#     canvas1.create_window(75, 25, window=tr.entry1)
#     tr.entry2 = tr.tk.Entry(tr.top,width=13)
#     canvas1.create_window(220,25, window=tr.entry2)

#     def gospec():
#         gx = int(tr.entry1.get())
#         gy = int(tr.entry2.get())

#         tr.program(gx,gy)
#     button1 = tr.tk.Button(tr.top,width=15, height=1, text='Move', command=gospec)
#     button1.place(x=325,y=140)
    
#     print ("X when motor is stop {}".format(x))
#     print ("Y when motor is stop {}".format(y))
#     tr.top.update()
    

    # mo.Motor.cubic_Polynominal(0,0,0,-20,2,0,0,0,0,0)  
def motor_run():
    global x, y
    count = 1
    while(True):
        xd, yd = tr.control()
        print("#{} times".format(count))
        print ("X when motor is stop {}".format(x))
        print ("Y when motor is stop {}".format(y))
        print ("X = {}".format(xd))
        print ("Y = {}".format(yd))
        count = count + 1
   




Thread(target = tracking).start()
Thread(target = motor_run).start()




