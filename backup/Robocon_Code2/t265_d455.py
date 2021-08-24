import pyrealsense2 as rs
import numpy as np

context = rs.context()
pipelines = []

for device in context.query_devices():
    pipe = rs.pipeline(context)
    config = rs.config()
    config.enable_device(device.get_info(rs.camera_info.serial_number))
    pipe.start(config)
    pipelines.append(pipe)
    print(device)

def grab_colour_frame():
    frames = pipelines[0].wait_for_frames()
    colour = np.asanyarray(frames.get_color_frame().get_data())
    return colour

def grab_pose_frame():
    frames = pipelines[1].wait_for_frames()
    pose = frames.get_pose_frame()
    return pose

while True:
    colour = grab_colour_frame()
    pose = grab_pose_frame()
   
    if pose:
        data = pose.get_data()
        print(data.translation)