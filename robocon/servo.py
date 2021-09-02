import time
import test
import TR_Move_Version6 as move
from adafruit_servokit import ServoKit
kit=ServoKit(channels=8)


def angle90():
    print('angle = 90')
    kit.servo[6].angle = 90

def angle0():
    print('angle = 0')
    kit.servo[6].angle = 0


def robot_move():
    while(True):
        print("Please input the value: ")
        id = int(input())
        if(id==1):
            cx,cy = move.program(115,5)
            # return 115, 5
        if(id==2):
            cx,cy = move.program(5,10)
            # return 5, 10
        elif(id==3):
            cx,cy = move.program(65,15)
            # return 65, 15
        elif(id==4):
            cx,cy = move.program(15,55)
            # return 15,55
    


