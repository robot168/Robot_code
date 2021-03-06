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
# import Object_Distance_Height as obj
from threading import Thread
import gyro_t265 as gy

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg) 
x = 115000
y = 5000
distance = 0


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
                x = (x + data.translation.z)/1000
                y = (y + (-data.translation.x))/1000
                x = round(x, 4)
                y = round(y, 4)
            
    finally:
        pipe.stop()   








