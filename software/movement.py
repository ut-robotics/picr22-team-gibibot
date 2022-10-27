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
            
    #def center_ball(self, xcord):
    #    if (xcord > 434):
    #        rotate_dir = -2
    #    elif (xcord < 414):
    #        rotate_dir = 2
    #    else:
    #        rotate_dir = 0
    #    print("Centering ball")
    #    self.comms.send_inf(rotate_dir,rotate_dir,rotate_dir,0,1)
        
    def move(self, speedX, speedR, speedY):
        #print("SAADAN ORBIT KIIRUSED")
        wheelx= self.calculator.calc_speed(speedX, speedY, speedR, 1)
        wheelr= self.calculator.calc_speed(speedX, speedY, speedR, 2)
        wheely= self.calculator.calc_speed(speedX, speedY, speedR, 3)
        

        self.comms.send_inf(wheelx, wheelr, wheely, 0, 1)

    def test_thrower(self, speedT):
        self.comms.send_inf(0,0,0,speedT,1)
        time.sleep(5)
        self.comms.send_inf(0,0,0,0,1)

    def throw(self, speedX, speedR, speedY, speedT):

        wheelx= self.calculator.calc_speed(speedX, speedY, speedR, 1)
        wheelr= self.calculator.calc_speed(speedX, speedY, speedR, 2)
        wheely= self.calculator.calc_speed(speedX, speedY, speedR, 3)

        self.comms.send_inf(wheelx, wheelr, wheely, speedT, 1)





        