
from os import system, name 
import sys
sys.path.append("..")
import numpy as np
import time
import math
import datetime as dt
from .environment import environment

import pybullet as p
from .spotmicroai import Robot
from .kinematicMotion import KinematicMotion,TrottingGait


rtime=time.time()
env=environment()

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

def settingimu(imux, imuy, iXf, spurWidth): ###
    timux = math.tan(imux)
    timuy = math.tan(imuy)
    rlegx = timux*spurWidth
    llegx = -timux*spurWidth
    flegy = timuy*(iXf+50)
    blegy = -timuy*50
    return rlegx, llegx, flegy, blegy

robot=Robot(True,False,reset)

spurWidth=robot.W/2+20
stepLength=0
stepHeight=72
iXf=120
iXb=-132
IDspurWidth = p.addUserDebugParameter("spur width", 0, robot.W, spurWidth)
IDstepHeight = p.addUserDebugParameter("step height", 0, 150, stepHeight)
IDheight = p.addUserDebugParameter("height", -40, 90, 40)

Lp = np.array([[iXf, -100,spurWidth, 1], [iXf, -100, -spurWidth, 1],
[-50, -100, spurWidth, 1], [-50, -100, -spurWidth, 1]])

resetPose()
trotting=TrottingGait()

def main(id, command_status):
    s=False
    while True:
        bodyPos=robot.getPos()
        bodyOrn,_,angularVel=robot.getIMU()
        xr,yr,_= p.getEulerFromQuaternion(bodyOrn)
        distance=math.sqrt(bodyPos[0]**2+bodyPos[1]**2)
        if distance>50:
            robot.resetBody()
   
        ir=xr/(math.pi/180)
        d=time.time()-rtime
        height = p.readUserDebugParameter(IDheight)

        result_dict = command_status.get()
        print(result_dict)
        command_status.put(result_dict)

        print(robot.getAngle()) ###
        rlegx, llegx, flegy, blegy= settingimu(xr, yr, iXf, spurWidth) ###
        
        if result_dict['StartStepping']:
            robot.feetPosition(trotting.positions(d-3, result_dict,bodyOrn))
        else:
            Lp = np.array([[iXf, -100 + llegx + flegy, spurWidth, 1], 
                        [iXf, -100 + rlegx + flegy, -spurWidth, 1],
                        [-50, -100 + llegx + blegy, spurWidth, 1], 
                        [-50, -100 + rlegx + blegy, -spurWidth, 1]])
                        ###
            robot.feetPosition(Lp)
        #roll=-xr

        roll=0
        robot.bodyRotation((roll,math.pi/180*((joy_x)-128)/3,-(1/256*joy_y-0.5)))
        bodyX=50+yr*10
        robot.bodyPosition((bodyX, 40+height, -ir))
        robot.step()
        consoleClear()


