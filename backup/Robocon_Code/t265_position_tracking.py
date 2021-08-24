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
import moveroot as mr
import Object_Distance_Height as obj
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
                x = data.translation.z
                y = -data.translation.x 
                x = round(x, 4)
                y = round(y, 4)
            
      
               
    
    finally:
        pipe.stop()   

    
def motor_run():
    global x, y
    count = 1
    while(True):
        xd, yd = tr.control()
        print("#{} times".format(count))
        print ("**From Camera")
        print ("X when motor is stop {}".format(x))
        print ("Y when motor is stop {}".format(y))
        print ("**From motor")
        print ("X = {}".format(xd))
        print ("Y = {}".format(yd))
        print ("**From Depth")
        print ("Distance is ", depthsensor())
        count = count + 1

def depthsensor():
    distance = obj.depth_s()  
    return distance


Thread(target = depthsensor).start()
Thread(target = tracking).start()
Thread(target = motor_run).start()






