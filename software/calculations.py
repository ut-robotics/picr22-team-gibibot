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
        t_dist=[53,72,94,118,143,165,190,222,248,290,310,350,390,430]
        t_speeds=[475,500,565,600,670,720,785,830,860,930,990,1050,1200,1300]
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

    def sig_correction_move(self, xcord, max_speed, a=1):

        base = np.linspace(-424, 424, 849)

        element = xcord - 19

        if element < 0:
            element = 0

        sigmoid = max_speed*2/(1 + np.exp(a*(-base)))

        speed = round((sigmoid[element] - max_speed), 2)

        print("keskpunkt kiirus - ", round((sigmoid[443] - max_speed), 2))

        return speed
    
    def sig_approach(self, ycord, max_speed, a=1):

        base = np.linspace(0, 480, 481)

        sigmoid = (max_speed*2)/(1 + np.exp(a*(-base)))

        sigmoid = np.flip(sigmoid)

        speed = abs(round((sigmoid[ycord]- max_speed), 2))

        return speed

    def sig_correction_orbit(self, ycord, max_speed, a=1):

        base = np.linspace(-52, 52, 105)

        element = ycord - 342

        if element < 0:
            element = 0
        elif element > 104:
            element = 104

        sigmoid = max_speed*2/(1 + np.exp(a*(-base)))

        speed = -(round((sigmoid[element] - max_speed), 2))

        return speed
        
        
        