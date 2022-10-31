from math import *
import numpy as np


class Calculations():
    def __init__(self):
        #Speed
        self.spin = 10
        self.wheel_from_centr=0.11
        self.pid_freq=60
        self.gearbox_reduction_ratio=18.75
        self.enc_edges_per_motor_rev=64
        self.wheel_angles=[240,0,120]
        self.wheel_radius=0.07
        self.disable_failsafe=1
        self.wheelSpeedTo_mu=self.gearbox_reduction_ratio*self.enc_edges_per_motor_rev/(2.0*pi*self.wheel_radius*self.pid_freq)
      

    def calc_speed(self, speedX, speedY, speedR, wheel_nr):
        robot_speed=sqrt(speedX * speedX + speedY * speedY)
        robotDirectionAngle = atan2(speedY, speedX)
        
        wheel_lin_speed=robot_speed*cos(robotDirectionAngle - np.radians(self.wheel_angles[wheel_nr - 1])) + self.wheel_from_centr * speedR
        wheel_ang_speed_mu=int(wheel_lin_speed * self.wheelSpeedTo_mu)
        ####print("MOVMENT CALC: ", wheel_lin_speed * self.wheelSpeedTo_mu)
    
        return  wheel_ang_speed_mu
    
    def calc_throwingSpeed(self, basketdist):
        tdist=[60,68,85,100,125,150,175,200,225,250,275,300,325,350,375,400,425]
        tspeeds=[472,472,500,520,550,620,670,730,780,810,865,900,930,985,1048,1115,1180]
        for i in range(len(tdist)):
            if basketdist<=tdist[i]:
                speedT1=(basketdist*tspeeds[i]/tdist[i])
                break
            
        for i in range(len(tdist)):
            if basketdist<=tdist[i]:
                if i!=0:
                    speedT2=(basketdist*tspeeds[i-1]/tdist[i-1])
                elif i==0:
                    speedT2=(basketdist*tspeeds[i]/tdist[i])
                break
        speedT=(speedT1+speedT2)/2 
        return speedT
        