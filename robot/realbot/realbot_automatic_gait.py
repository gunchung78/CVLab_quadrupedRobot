
from ctypes import sizeof
from os import system, name 
import sys
sys.path.append("..")
import numpy as np
import time
import math
import datetime as dt

from .spotmicroai import Robot
from .kinematicMotion import KinematicMotion,TrottingGait
import serial
from .serial2servo import serial_servo


try:
    PORT = 'COM6' #포트이름
    BaudRate = 115200 #전송속도
    ard = serial.Serial(PORT,BaudRate)
    uart_bool = True
except:
    uart_bool = False
    pass

rtime=time.time()

def reset():
    global rtime
    rtime=time.time()    

def resetPose():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz,joy_z
    joy_x, joy_y, joy_z, joy_rz = 128, 128, 128, 128

def consoleClear():
    # for windows 
    if name == 'nt': 
        _ = system('cls') 
  
    # for mac and linux(here, os.name is 'posix') 
    else: 
        _ = system('clear') 

robot=Robot(False,False,reset)

# 스탠딩 값
spurWidth=robot.W/2+20
iXf=120 
IDheight = 40 

Lp = np.array([[iXf, -100,spurWidth, 1], [iXf, -100, -spurWidth, 1],
[-50, -100, spurWidth, 1], [-50, -100, -spurWidth, 1]])

resetPose()
trotting=TrottingGait()

def main(id, command_status):
    s=False
    while True:
        # bodyPos=robot.getPos()
        # bodyOrn,_,angularVel=robot.getIMU()
        #xr,yr,_= p.getEulerFromQuaternion(bodyOrn)
        xr = 0
        yr = 0
        # distance=math.sqrt(bodyPos[0]**2+bodyPos[1]**2)
        # if distance>50:
        #     robot.resetBody()
   
        ir=xr/(math.pi/180)
        d=time.time()-rtime
        height = IDheight

        result_dict = command_status.get()
        print(result_dict)
        command_status.put(result_dict)
        
        if result_dict['StartStepping']:
            robot.feetPosition(trotting.positions(d-3, result_dict))
        else:
            robot.feetPosition(Lp)

        roll=0
        robot.bodyRotation((roll,math.pi/180*((joy_x)-128)/3,-(1/256*joy_y-0.5)))
        bodyX=50+yr*10
        robot.bodyPosition((bodyX, 40+height, -ir))
        robot.step()
        
        #pyserial part
        ser = serial_servo(180, 1)
        allangle =  robot.getAngle()*(180/math.pi)
        print(np.round(allangle,3))
        if uart_bool:
             for val1 in range(4):
                 for val2 in range(3):
                    angl_float = 90 + allangle[val1][val2]
                    angle_byte = ser.angle2byte(1, angl_float)
                    ard.write(angle_byte[0])
                    ard.write(angle_byte[1]) 
        else:
            print("uart: x")
        

        consoleClear()


