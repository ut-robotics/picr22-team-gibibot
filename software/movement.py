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
        self.ball_in_grabber = 0
    
    
    def stop(self):
        self.comms.send_inf(0,0,0,0,6900,4800)

    def try_motors(self):
        self.comms.send_inf(-40,-40,-40,0,6900,4800)
    
    def find_ball(self, rotate_speed):
        self.comms.send_inf(rotate_speed,rotate_speed,rotate_speed,0,6900,4800)
        
    def move(self, speed_X, speed_R, speed_Y,speed_T, thrower_angle, grabber):
        wheelx= self.calculator.calc_speed(speed_X, speed_Y, speed_R, 1)
        wheelr= self.calculator.calc_speed(speed_X, speed_Y, speed_R, 2)
        wheely= self.calculator.calc_speed(speed_X, speed_Y, speed_R, 3)
        
        self.comms.send_inf(wheelx, wheelr, wheely, speed_T, thrower_angle, grabber)
        self.ball_in_grabber = self.comms.ball_in_grabber

    def test_thrower(self, speed_T, speed_Y):
        self.move(0,0,speed_Y,speed_T,6900,6900)
        time.sleep(2)
        self.comms.send_inf(0,0,0,0,6900,4800)

    def straight_movement(self, speed_Y):
        self.move(0,0,speed_Y,0,6900,4800)
    
    def side_movement(self, speed_X):
        self.move(speed_X,0,0,0,6900,4800)

    





        