import numpy as np 

def sig(xcord, max_speed_X, max_speed_Y, max_speed_R, a=1):

    frame_width = np.linspace(-424, 424, 848)

    sigmoid_X = max_speed_X*2/(1 + np.exp(a*(-frame_width)))

    sigmoid_Y = max_speed_Y*2/(1 + np.exp(a*(-frame_width)))

    sigmoid_R = max_speed_R*2/(1 + np.exp(a*(-frame_width)))

    speed_X = sigmoid_X[xcord] - max_speed_X
    speed_Y = sigmoid_Y[xcord] - max_speed_Y
    speed_R = sigmoid_R[xcord] - max_speed_R

    return speed_X, speed_Y, speed_R



xcord = 200
a = 0.01
max_speed=1.5
p = sig(xcord, max_speed, a)
print(p)