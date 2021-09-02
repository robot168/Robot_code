import serial, time
import os 
# import servo
import Jetson.GPIO as GPIO

import math 
# from adafruit_servokit import ServoKit
# kit=ServoKit(channels=16)

arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
time.sleep(1) #give the connection a second to settle
# arduino.write(str.encode("Hello from Python!"))
encoder = 0 
encoder_value = 0
c_angle = 0
c_angle_value = 0 
i_angle = 0

def trans(replysize, data= bytearray()):
    respon = [0]*replysize
    arduino.write(data)       
    global encoder_value
    global c_angle
    global c_angle_value

    while(arduino.in_waiting==0):
        # print("wait")
        continue
    while(arduino.inWaiting()>0):
        arduino.readinto(respon)
    for j in range(0,replysize):
        print(hex(respon[j]))
    if (encoder == 1):
        encoder_value = ((respon[8]<<8)|respon[7])
    if (c_angle == 1):
        for i in range(5, 13):
            c_angle_value = ((respon[i]<<8*(i-5)) | c_angle_value) 
        


def stopmotor(idm):
    mstop = bytearray(5)
    mstop[0] = 0x3e
    mstop[1] = 0x81
    mstop[2] = idm
    mstop[3] = 0
    mstop[4] = (mstop[0]+mstop[1]+mstop[2]+mstop[3]) & 0x00ff
    trans(5,mstop)

def opmotor(idm):
    mop = bytearray(5)
    mop[0] = 0x3e
    mop[1] = 0x88
    mop[2] = idm
    
    mop[3] = 0x00
    mop[4] = (mop[0]+mop[1]+mop[2]+mop[3]) & 0x00ff
    trans(5,mop)

def runmotorspeed(idm, speed):
    mtspeed = round(speed,2)*100

    mrunspd = bytearray(10)
    mrunspd[0] = 0x3e
    mrunspd[1] = 0xa2
    mrunspd[2] = idm
    mrunspd[3] = 0x04
    mrunspd[4] = (mrunspd[0]+mrunspd[1]+mrunspd[2]+mrunspd[3]) & 0x00ff
    for i in range(5,9):
        mrunspd[i] = (mtspeed>>8*(i-5)) & 0x000000ff
    mrunspd[9] = (mrunspd[5]+mrunspd[6]+mrunspd[7]+mrunspd[8]) & 0x00ff 
    trans(13,mrunspd)

    
def runmotor_angle(idm, angle, speed):
    mtangle = (angle)*100
    mtspeed = round(speed,2)*100


    mrunang = bytearray(18)
    mrunang[0] = 0x3e
    mrunang[1] = 0xa4
    mrunang[2] = idm
    mrunang[3] = 0x0c
    mrunang[4] = (mrunang[0]+mrunang[1]+mrunang[2]+mrunang[3]) & 0x00ff
    mrunang[17] = 0
    for i in range(5,13):
        mrunang[i] = (mtangle>>8*(i-5)) & 0x00000000000000ff
    for i in range(13,17):
        mrunang[i] = (mtspeed>>8*(i-13)) & 0x000000ff

    for i in range(5,17):
        mrunang[17] =( mrunang[17] + mrunang[i]) & 0x00ff
    # mrunang[17] = mrunang[17] & 0x00ff
        
    trans(13,mrunang)

def readencoder(idm):
    mencoder = bytearray(5)
    mencoder[0] = 0x3e
    mencoder[1] = 0x90
    mencoder[2] = idm
    mencoder[3] = 0
    mencoder[4] = (mencoder[0]+mencoder[1]+mencoder[2]+mencoder[3]) & 0x00ff
    global encoder
    encoder = 1
    trans(12,mencoder)
    global encoder_value
    return encoder_value

def writeMotor0 (idm):
    global encoder
    x = readencoder(idm)
    encoder = 0
    print('x = ',x)
    time.sleep(1)
    m0 = bytearray(8)
    m0[0] = 0x3e
    m0[1] = 0x91
    m0[2] = idm
    m0[3] = 0x02
    m0[4] = (m0[0]+m0[1]+m0[2]+m0[3]) & 0x00ff
    m0[5] = x & 0x00ff
    m0[6] = x>>8 
    m0[7] =(m0[5] + m0[6]) & 0x00ff
    trans(8, m0)
    stopmotor(idm)
    runmotor_angle(idm, 1, 100)


def readAngle (idm):
    ma = bytearray(5)
    global c_angle_value
    global c_angle

    c_angle = 1
    ma[0] = 0x3e
    ma[1] = 0x92
    ma[2] = idm
    ma[3] = 0x00
    ma[4] = (ma[0]+ma[1]+ma[2]+ma[3]) & 0x00ff
    trans(14, ma)
    result = c_angle_value
    c_angle = 0
    print(result)
    if(result > 100000):
        x = bin(result)
        N = int(x, 2)
        # this is the two complements
        TC = (-1)*(18446744073709551615 + 1 - N)
        # TC = TC*(-1)
        result = TC/100
    else:
        result = result / 100
    return result






GPIO.cleanup(15)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(15, GPIO.IN)

runmotorspeed(5, -10)
try:
    print("Waiting for button event")
    # GPIO.wait_for_edge(15, GPIO.FALLING)
    print('GPIO = ', GPIO.input(15))

    while (GPIO.input(15) == 1):
        time.sleep(0.001)
        print('helloooo')

    print('GPIO2 = ', GPIO.input(15))
    print("Button Pressed!")
    stopmotor(5)
    i_angle = readAngle(5)
    print(i_angle)

finally:
    GPIO.cleanup()  # cleanup all GPIOs

    