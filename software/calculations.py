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
      

    def calc_speed(self, speed_X, speed_Y, speed_R, wheel_nr):
        robot_speed=sqrt(speed_X * speed_X + speed_Y * speed_Y)
        robotDirectionAngle = atan2(speed_Y, speed_X)
        
        wheel_lin_speed=robot_speed*cos(robotDirectionAngle - np.radians(self.wheel_angles[wheel_nr - 1])) + self.wheel_from_centr * speed_R
        wheel_ang_speed_mu=int(wheel_lin_speed * self.wheelSpeedTo_mu)
        ####print("MOVMENT CALC: ", wheel_lin_speed * self.wheelSpeedTo_mu)
    
        return  wheel_ang_speed_mu
    
    def calc_throwingSpeed(self, basket_dist):
        t_dist=[60,68,85,100,125,150,175,200,225,250,275,300,325,350,375,400,425]
        t_speeds=[472,472,500,520,550,620,670,730,780,810,865,900,930,985,1048,1115,1180]
        desired_speed=0
        for i in range(len(t_dist)):
            if basket_dist<=t_dist[i]:
                
                #speed_T1=(basket_dist*t_speeds[i]/t_dist[i])
                if i!=0:
                    percentage=(basket_dist-t_dist[i-1])/(t_dist[i]-t_dist[i-1])
                    #speed_T2=(basket_dist*t_speeds[i-1]/t_dist[i-1])
                elif i==0:
                    desired_speed=(basket_dist*t_speeds[i]/t_dist[i])
                    return desired_speed
                
                desired_speed=t_speeds[i-1]+(t_speeds[i]-t_speeds[i-1])*percentage
                
                return desired_speed
        #speed_T=(speed_T1+speed_T2)/2 
        return desired_speed
        