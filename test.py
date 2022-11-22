import numpy as np
from random import randint

x = 75
y = 100
h = 30
w = 35

frame_height = 150
frame_width = 200

tabel = []
for i in range(frame_height):
    x_rida = []
    for i in range(frame_width):
        a = randint(0, 100)
        x_rida.append(a)
    tabel.append(x_rida)

tabel = np.array(tabel)

print("FRAME ON SELLINE: ", tabel)



ys = np.array(np.arange(y+h,frame_height),dtype=np.uint16)
print("ys: ", ys)

xs = np.array(np.linspace(x + w/2,frame_width/2, num=len(ys)),dtype=np.uint16)
print("xs: ", xs)

joon = tabel[ys,xs]

print("Joon pallist frame elementidega ehk varvsequence: ", joon)