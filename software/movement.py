import math
from turtle import distance
import numpy as np
import time
import serial
import tkinter as tk
import struct
from math import *

disable_failsafe=0


class OmniRobot:
    def __init__(self):
        global ser
        ser = serial.Serial()
        ser.port='/dev/ttyACM0'
        ser.baudrate=9600
        ser.open()
        print("Serial opened")
        
        #Speed
        self.spin = 10
        self.wheel_from_centr=0.11
        self.pid_freq=60
        self.gearbox_reduction_ratio=18.75
        self.enc_edges_per_motor_rev=64
        self.wheel_angles=[240,0,120]
        self.wheel_radius=0.07
        #self.wheel_lin_speed=2
        self.disable_failsafe=1
        self.wheelSpeedTo_mu=self.gearbox_reduction_ratio*self.enc_edges_per_motor_rev/(2.0*math.pi*self.wheel_radius*self.pid_freq)
        #self.wheelAngularSpeed_mu = self.wheelSpeedTo_mu * self.wheel_lin_speed 
        
        #wheelSpeedToMainboardUnits = gearboxReductionRatio * encoderEdgesPerMotorRevolution / (2 * PI * wheelRadius * pid_freq)
        #wheelAngularSpeedMainboardUnits = wheelLinearVelocity * wheelSpeedToMainboardUnits
    def close(self):
        ser.close()
        print("Serial closed")
    
    
        
    
    def send_inf(self, speed1, speedr, speed3, thrower_speed, disable_failsafe):
        inf_out=struct.pack('<hhhHBH', speed1, speedr, speed3, thrower_speed, disable_failsafe, 0xAAAA)
        ser.write(inf_out)
        print("Data sent")
        inf_in=ser.read(8)
        actual_speed1, actual_speedr, actual_speed3, feedback_delimiter = struct.unpack('<hhhH', inf_in)
        print("Real Data: " , actual_speed1, actual_speedr, actual_speed3, feedback_delimiter)

    def communication_test(self):
        self.send_inf(0,0,0,0,0)
        
    
    def stop(self):
        self.send_inf(0,0,0,0,1)

    def try_motors(self):
        self.send_inf(10,10,10,0,0)
    
    def omni(self, xcord,wheel_nr, distance):
        delta=((xcord-424)/424)
        speedR = delta*(-1.75)
        speedX = delta*0.25
        
        if distance > 300:
            speedY=0.1
        else:
            speedY=0.5

        
        robot_speed=sqrt(speedX * speedX + speedY * speedY)
        robotDirectionAngle = atan2(speedY, speedX)

        wheel=int(self.calc_speed(robot_speed, robotDirectionAngle, wheel_nr, speedR))
        return wheel

    def calc_speed(self, robotSpeed, robotAngle, m_number, speed_r):
        wheel_lin_speed=robotSpeed*math.cos(robotAngle - np.radians(self.wheel_angles[m_number - 1])) + self.wheel_from_centr * speed_r
        wheel_ang_speed_mu=wheel_lin_speed * self.wheelSpeedTo_mu
        return wheel_ang_speed_mu


    ## Movement for securing and finding the ball

    def one_meter(self):

        start_time = time.time()

        while(self.timer(3, start_time)):
            self.send_inf(self.omni(0, 0.5, 1), self.omni(0, 0.5, 2), self.omni(0, 0.5, 3), 0, 1)
        self.stop()
    
    def find_ball(self, rotate_speed, ball_count):
        print("rotate")
        self.send_inf(rotate_speed,rotate_speed,rotate_speed,0,1)
            

    def center_ball(self, xcord):
        if (xcord > 434):
            rotate_dir = -2
        elif (xcord < 414):
            rotate_dir = 2
        else:
            rotate_dir = 0
        print("Centering ball")
        self.send_inf(rotate_dir,rotate_dir,rotate_dir,0,1)

    def secure_ball(self, distance):
        if (True):
            self.send_inf(0, 0, 1, 0, 1)

    def timer(self, sec, start_time):
        run_time = time.time() - start_time

        if run_time > sec:
            print("Shutting down.", run_time)
            return 0
        else:
            return 1


        