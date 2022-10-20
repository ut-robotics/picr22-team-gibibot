import math
import numpy as np
import time
import serial
import tkinter as tk
import struct
from math import *
import communication

disable_failsafe=0


class OmniRobot:
    def __init__(self):
        self.comms=communication.Communication()
    
    
    def stop(self):
        self.comms.send_inf(0,0,0,0,1)

    def try_motors(self):
        self.comms.send_inf(10,10,10,0,0)
    
    def find_ball(self, rotate_speed):
        print("rotate")
        self.comms.send_inf(rotate_speed,rotate_speed,rotate_speed,0,1)
            
    def center_ball(self, xcord):
        if (xcord > 434):
            rotate_dir = -2
        elif (xcord < 414):
            rotate_dir = 2
        else:
            rotate_dir = 0
        print("Centering ball")
        self.comms.send_inf(rotate_dir,rotate_dir,rotate_dir,0,1)
        
    def orbit(self, wheelx, wheelr, wheely):
        #print("SAADAN ORBIT KIIRUSED")
        

        self.comms.send_inf(wheelx, wheelr, wheely, 0, 1)




        