import math
import numpy as np
import time
import serial
import tkinter as tk
import struct
from math import *
import communication
import calculations

disable_failsafe=0


class OmniRobot:
    def __init__(self):
        self.comms=communication.Communication()
        self.calculator=calculations.Calculations()
    
    
    def stop(self):
        self.comms.send_inf(0,0,0,0,1)

    def try_motors(self):
        self.comms.send_inf(10,10,10,0,0)
    
    def find_ball(self, rotate_speed):
        self.comms.send_inf(rotate_speed,rotate_speed,rotate_speed,0,1)
        
    def move(self, speed_X, speed_R, speed_Y,speed_T):
        wheelx= self.calculator.calc_speed(speed_X, speed_Y, speed_R, 1)
        wheelr= self.calculator.calc_speed(speed_X, speed_Y, speed_R, 2)
        wheely= self.calculator.calc_speed(speed_X, speed_Y, speed_R, 3)
        

        self.comms.send_inf(wheelx, wheelr, wheely, speed_T, 1)

    def test_thrower(self, speed_T):
        self.comms.send_inf(0,0,0,speed_T,1)
        time.sleep(3)
        self.comms.send_inf(0,0,0,0,1)





        