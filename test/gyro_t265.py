import pyrealsense2 as rs
import numpy as np
import time

gyro_sensor = 0

def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    prof = p.start(conf)
    return p


def gyro_data(gyro):
    global gyro_sensor
    gyro_sensor = gyro.y
    return gyro.y


# def accel_data(accel):
#     return np.asarray([accel.x, accel.y, accel.z])

# p = initialize_camera()
# try:
#     while True:
#         f = p.wait_for_frames()
#         # accel = accel_data(f[1].as_motion_frame().get_motion_data())
#         gyro = gyro_data(f[0].as_motion_frame().get_motion_data())
#         # print("accelerometer: ", accel)
#         print("gyro: ", gyro)
#         time.sleep(1)

# finally:
#     p.stop()

p = initialize_camera()
try:
    while True:
        f = p.wait_for_frames()
        gyro_senso = gyro_data(f[0].as_motion_frame().get_motion_data())
finally:
    p.stop()