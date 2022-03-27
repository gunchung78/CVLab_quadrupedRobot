import numpy as np
from math import *




def kin(x,y,z):
    l1=34
    l2=100
    l3=124

    xyl1=x**2+y**2-l1**2
    angle_1=-atan2(-y,x)-atan2(sqrt(xyl1),-l1)
    D=(xyl1+z**2-l2**2-l3**2)/(2*l2*l3)
    angle_3=atan2(sqrt(1-D**2),D)
    angle_2=atan2(z,sqrt(xyl1))-atan2(l3*sin(angle_3),l2+l3*cos(angle_3))
    return (angle_1,angle_2,angle_3)

def legIK(x,y,z):
    l1=34
    l2=0
    l3=100
    l4=124

    try:        
        F=sqrt(x**2+y**2-l1**2)
    except ValueError:
        print("Error in legIK with x {} y {} and l1 {}".format(x,y,l1))
        F=l1
    G=F-l2  
    H=sqrt(G**2+z**2)
    theta1=-atan2(-y,x)-atan2(F,-l1)
    
    D=(H**2-l3**2-l4**2)/(2*l3*l4)
    try:        
        theta3=acos(D) 
    except ValueError:
        print("Error in legIK with x {} y {} and D {}".format(x,y,D))
        theta3=0
    theta2=atan2(z,G)-atan2(l4*sin(theta3),l3+l4*cos(theta3))
    
    return(theta1,theta2,theta3)

if __name__=="__main__":
    print(kin(100,100,100))
    print(legIK(100,100,100))