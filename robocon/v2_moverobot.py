from termios import TAB1
from numpy import byte, int32
from numpy.core.records import array
import serial, time

import os

import matplotlib.pyplot as plt

import math 
arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
print("serial open")
time.sleep(1) #give the connection a second to settl
vxo = 0
vyo = 0
vxfin = 1
vyfin = 1
readSpeed = bytearray(13)
start=1
s1 = []
s2 = []
s3 = []
    
def trans(replysize, data= bytearray()):
    global readSpeed
    respon = [0]*replysize
    arduino.write(data)
    while(arduino.in_waiting==0):
        # print("wait")
        continue
    while(arduino.inWaiting()>0):
        arduino.readinto(respon)
    for j in range(0,replysize):
        hex(respon[j])
        
    if(replysize == 13):
        readSpeed = respon
        # print (readSpeed)

def stopmotor(idm):
    mstop = bytearray(5)
    mstop[0] = 0x3e
    mstop[1] = 0x81
    mstop[2] = idm
    mstop[3] = 0
    mstop[4] = (mstop[0]+mstop[1]+mstop[2]+mstop[3]) & 0x00ff
    trans(5,mstop)
        
def readSpeed(mId):
    RmSp = bytearray(5)
    result = int
    RmSp[0] = 0x3e
    RmSp[1] = 0x9c
    RmSp[2] = mId
    RmSp[3] = 0x00
    RmSp[4] = (RmSp[0]+RmSp[1]+RmSp[2]+RmSp[3]) & 0x00ff
        
    trans(13,RmSp)
    if readSpeed[0] == 62 and readSpeed[1] == 156 and readSpeed[2] == mId:
        result = (readSpeed[9] << 8) | (readSpeed[8])
        # print("result = ", result)
        if(result > 10000):
            x = bin(result)
            N = int(x, 2)
            # this is the two complements
            TC = (-1)*(65535 + 1 - N)
            result = TC*(-1)
    return result

       
        
        
        
def opmotor(idm):
    mop = bytearray(5)
    mop[0] = 0x3e
    mop[1] = 0x88
    mop[2] = idm
        
    mop[3] = 0x00
    mop[4] = (mop[0]+mop[1]+mop[2]+mop[3]) & 0x00ff
    trans(5,mop)

def runmotorspeed(idm, speed):
        
    mtspeed = speed

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

def movemotor(x, y, d):
    R = 3.5
    r = 0.76
    idm1 = 1 #motor id
    idm2 = 2 #motor id
    idm3 = 3 #motor id

    #speed motor
    motor1 = (((-math.sin(math.radians(60))*math.cos(math.radians(60)))*x) + ((math.cos(math.radians(60))*math.cos(math.radians(60)))*y) + (R*d))/r
    motor1 = math.degrees(motor1) 
    motor1= int(motor1*100)
        # print(" motor1", motor1)

    motor2 = (((-math.sin(math.radians(180))*math.cos(math.radians(60)))*x) + ((math.cos(math.radians(180))*math.cos(math.radians(60)))*y) + (R*d))/r
    motor2 = math.degrees(motor2)
    # motor2 = int(motor2*100)/
    motor2 = int(motor2*100) 
    # print(" motor2", motor2)

    motor3 = (((-math.sin(math.radians(300))*math.cos(math.radians(60)))*x) + ((math.cos(math.radians(300))*math.cos(math.radians(60)))*y) + (R*d))/r
    motor3 = math.degrees(motor3)
    motor3 = int(motor3*100) 
    # print("motor3", motor3)
        

     
    global start
    runmotorspeed(idm1,motor1)
    runmotorspeed(idm2,motor3)
    runmotorspeed(idm3,motor2)    


def cubic_Polynominal(x0, x1, y0, y1, tf, vx0, vxf, vy0, vyf, d):
    a0x = x0
    a0y = y0
    a1x = vx0        
    a1y = vy0
    tf = tf*0.5
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
    tf1 =int(tf*270)
    for t in range(tf1+1):
        time1= time.perf_counter()     
        # print("t=", t)
        vx = a1x+ (2*a2x*(t/270)) + (3*a3x*(t/270)*(t/270))
        vy = a1y+ (2*a2y*(t/270)) + (3*a3y*(t/270)*(t/270))
        if(x1==x0):
            vx=0
        if(y1==y0):
            vy=0
        # print("vx", vx)
        # print("vy", vy)
        # print("d", d)
        d = tmp - (d*t)/(tf1)
        # print (d)
        movemotor(vx, vy, d);
        # time.sleep(0.012)
        if(t == tf1):
            #time.sleep(0.72)
            stopmotor(1)
            stopmotor(2)
            stopmotor(3)
            
        #s2.append(readSpeed(2))
        #s1.append(readSpeed(1))
        #s3.append(readSpeed(3))
            
            
         
           


# Motor.runmotorspeed(2,20000)
# while(True):
#     print("Please input the value: ")
#     x = int(input())
#     if(x==1):
#         cubic_Polynominal(0,20,0,20,4,0,0,0,0,0)
#     elif(x==2):
#         cubic_Polynominal(0,-20,0,-20,4,0,0,0,0,0)
#     elif(x==3):
#         cubic_Polynominal(0,0,0,20,4,0,0,0,0,0)
#     elif(x==4):
#         cubic_Polynominal(0,0,0,-20,4,0,0,0,0,0)        
# i = 1
# for i in range(3):
#     time.sleep(2)
#     Motor.cubic_Polynominal(0,40,0,0,4,0,0,0,0,0)
#     time.sleep(2)
#     Motor.cubic_Polynominal(0,-40,0,0,4,0,0,0,0,0)
# while(True):
#     print("Please input the value: ")
#     id = int(input())
#     if(id==1):
#         Motor.cubic_Polynominal(10,0,10,0,1.5,0,0,0,0,0)  
#         print(s1)
#         print(s2)
#         print(s3)
#         plt.plot(s1, "-r")
#         plt.plot(s2, "-g")
#         plt.plot(s3, "-b")
#         #plt.pause(0.01)
#         plt.show()
#         continue 
#     elif (id==2):
#         Motor.cubic_Polynominal(-10,0,-10,0,1.5,0,0,0,0,0)  
#         print(s1)
#         print(s2)
#         print(s3)
#         plt.plot(s1, "-r")
#         plt.plot(s2, "-g")
#         plt.plot(s3, "-b")
#         #plt.pause(0.01)
#         plt.show()
#         continue
#     elif (id==3):
#         Motor.cubic_Polynominal(50,0,0,0,5,0,0,0,0,0)  
#         print(s1)
#         print(s2)
#         print(s3)
#         plt.plot(s1, "-r")
#         plt.plot(s2, "-g")
#         plt.plot(s3, "-b")
#         #plt.pause(0.01)
#         plt.show()
#     elif (id==4):
#         Motor.cubic_Polynominal(-50,0,0,0,5,0,0,0,0,0)  
#         print(s1)
#         print(s2)
#         print(s3)
#         plt.plot(s1, "-r")
#         plt.plot(s2, "-g")
#         plt.plot(s3, "-b")
#         #plt.pause(0.01)
#         plt.show()

# arduino.close()
# while(True):
    # print("Please input the number: ")
    # id = int(input())
    # if(id==1):
    #     Motor.cubic_Polynominal(0, 10, 0, 0, 1.5, 0, 0, 0, 0, 0)

    # elif(id==2):
    #     Motor.cubic_Polynominal(0, 20, 0, 0, 3, 0, 0, 0, 0, 0)

    # elif(id==3):
    #     Motor.cubic_Polynominal(0, 0, 0, 10, 1.5, 0, 0, 0, 0, 0)

    # elif(id==4):
    #     Motor.cubic_Polynominal(0, 0, 0, 20, 3, 0, 0, 0, 0, 0)

    # elif(id==5):
    #     Motor.cubic_Polynominal(0, 10, 0, 10, math.sqrt(2), 0, 0, 0, 0, 0)

    # elif(id==6):
    #     Motor.cubic_Polynominal(0,20, 0, 20, 2*math.sqrt(2), 0, 0, 0, 0, 0)
