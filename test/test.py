import gyro_t265 as gy


p = gy.initialize_camera()
try:
    while True:
        f = p.wait_for_frames()
        gyro_sensor = gy.gyro_data(f[0].as_motion_frame().get_motion_data())
        print (gyro_sensor)
finally:
    p.stop()
