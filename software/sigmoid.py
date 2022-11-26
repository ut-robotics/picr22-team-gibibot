import numpy as np 

def sig_correction(xcord, max_speed, a=1):

    base = np.linspace(-424, 424, 847)

    sigmoid = max_speed*2/(1 + np.exp(a*(-base)))

    speed = round((sigmoid[xcord] - max_speed), 2)

    return speed

def sig_approach(ycord, max_speed, a=1):

    base = np.linspace(-480, 0, 481)

    sigmoid = -(max_speed*2)/(1 + np.exp(a*(-base)))

    sigmoid = np.flip(sigmoid)

    speed = abs(round(sigmoid[ycord], 2))

    return speed



xcord = 624
ycord = 0
a = 0.01
max_speed_X=1.0
max_speed_Y=1.8
max_speed_R=0.8

speed_X = sig_correction(xcord,max_speed_X, a)

speed_Y = sig_approach(ycord,max_speed_Y, a)

speed_R = sig_correction(xcord,max_speed_R, a)

print("speedX: ", speed_X)
print("speedY: ", speed_Y)
print("speedR: ", speed_R)