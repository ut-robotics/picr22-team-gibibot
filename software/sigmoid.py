import numpy as np 

def sig(xcord, max_speed, a=1):

    frame_width = np.linspace(-424, 424, 848)

    sigmoid = max_speed*2/(1 + np.exp(a*(-frame_width)))

    speed = round((sigmoid[xcord] - max_speed), 2)

    return speed



xcord = 424
a = 0.01
max_speed_R=0.8
speed_R = sig(xcord,max_speed_R, a)
print("speedR: ", speed_R)