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
        self.wheel_angles=[240,120,0]
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
        t_dist=[69,93,109,117,124,138,150,161,172,180,191,206,215,225,240,255,278,301,310,322,345,362,400,420,430]
        t_speeds=[900,975,1015,1075,1090,1145,1190,1225,1250,1285,1310,1350,1390,1430,1485,1525,1650,1700,1775,1800,1855,1990,2050,2125,2175]
        desired_speed=0
        for i in range(len(t_dist)):
            if basket_dist<=t_dist[i]:
                
                if i!=0:
                    percentage=(basket_dist-t_dist[i-1])/(t_dist[i]-t_dist[i-1])
                elif i==0:
                    desired_speed=(basket_dist*t_speeds[i]/t_dist[i])
                    return desired_speed
                
                desired_speed=t_speeds[i-1]+(t_speeds[i]-t_speeds[i-1])*percentage
                
                return desired_speed
        return desired_speed

    def sig_correction_move(self, xcord, max_speed, a=1):

        base = np.linspace(-424, 424, 849)

        element = xcord + 0
        
        element = max(element, 0)
        element = min(element, 848)

        sigmoid = max_speed*2/(1 + np.exp(a*(-base)))

        speed = round((sigmoid[element] - max_speed), 2)

        return speed
    
    def sig_approach(self, ycord, max_speed, a=1):

        base = np.linspace(0, 400, 481)

        sigmoid = (max_speed*2)/(1 + np.exp(a*(-base)))

        sigmoid = np.flip(sigmoid)

        speed = abs(round((sigmoid[ycord]- max_speed), 2))

        return speed

    def sig_correction_orbit(self, ycord, max_speed, a=1):

        base = np.linspace(-52, 52, 105)

        element = ycord - 342
        
        element = max(element, 0)
        element = min(element, 104)

        sigmoid = max_speed*2/(1 + np.exp(a*(-base)))

        speed = -(round((sigmoid[element] - max_speed), 2))

        return speed
        
        
        
