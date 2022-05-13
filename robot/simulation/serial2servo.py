import serial
import time

class serial_servo:
    def __init__(self, maxangle, version):
        self.maxangle = maxangle
        self.version = version

    def angle2byte(self, num, angle):
        if angle > self.maxangle and angle < 0:
            print("angle X")
        else:
            if self.version == 1:
                angle_int = int(angle)
                angle_float = int(round(angle- angle_int, 2)*100)
                byte_angleint = bytes([angle_int])
                byte_anglefloat = bytes([angle_float])
            elif self.version == 2:
                print("x")
            return byte_angleint, byte_anglefloat


if __name__ == "__main__":
    ard = serial.Serial('COM6', 115200, timeout = 1)
    ser = serial_servo(180, 1)
    while True:
        print("[input angle]")
        inangle = float(input())
        if inangle >= 0 and inangle <= 180:
            angle = ser.angle2byte(1, inangle)
            ard.write(angle[0])
            ard.write(angle[1]) 
            print(angle[0],angle[1])
            # res = ard.readline()
            # print("ard: " + res.decode()[:len(res)-1])
        else:
            print("angle x")

        



        

