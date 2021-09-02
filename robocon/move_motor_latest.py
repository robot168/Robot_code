import serial, time
import os 
import Jetson.GPIO as GPIO

import math 

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


def move_wheel(x, y, d):
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

def cubic_Polynominal(x0, x1, y0, y1, tf, vx0, vxf, vy0, vyf, d):
    a0x = x0
    a0y = y0
    a1x = vx0        
    a1y = vy0
    tf = tf*0.75
    a2x = ((3*(x1-x0))/(tf*tf)) - ((2*vx0)/tf) - (vxf/tf)
    a2y = ((3*(y1-y0))/(tf*tf)) - ((2*vy0)/tf) - (vyf/tf)
    a3x = ((-2/(tf*tf*tf))*(x1-x0)) + ((vxf+vx0)/(tf*tf))
    a3y = ((-2/(tf*tf*tf))*(y1-y0)) + ((vyf+vy0)/(tf*tf))
    global vxo
    global vyo
    global vxfin
    global vyfin
    global s1, s2, s3
    tmp = d
    t = 0
    tf1 =int(tf*100)
    for t in range(tf1+1):
        vx = a1x+ (2*a2x*(t/100)) + (3*a3x*(t/100)*(t/100))
        vy = a1y+ (2*a2y*(t/100)) + (3*a3y*(t/100)*(t/100))
        if(x1==x0):
            vx=0
        if(y1==y0):
            vy=0 
        d = tmp - (d*t)/(tf1)
        move_wheel(vx, vy, d);
        time.sleep(0.00079)
        if(t == tf1):
            stopmotor(1)
            stopmotor(2)
            stopmotor(3)
        
        # s2.append(readSpeed1(2))
        # s1.append(readSpeed1(1))
        # s3.append(readSpeed1(3))
    
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
    c_angle = 0
    result = c_angle_value
    # print(result)
    if(result > 1000000000):
        x = bin(result)
        N = int(x, 2)
        # this is the two complements
        TC = (-1)*(18446744073709551615 + 1 - N)
        result = TC/100
        # result = TC*(-1)
    else:
        result = result / 100
    return result 
