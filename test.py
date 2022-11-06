import numpy as np

x = np.array([[3, 5, 2],
              [2, 3, 3],
              [1, 6, 3],
              [1, 1, 1],
              [3, 4, 1],
              [3, 3, 3], 
              [2 , 5, 6]])

x[x != 3] = 0

WHITE = 5, np.array([255, 255, 255], dtype=np.uint8)
#return value on 2 arraid millest esimene annab eile ifo koigid y listide kohta kus valued ei ole o ja teiene list mis annab meile nedes
#listides olevate x elementide indexid mis ei ole nullid
#print(np.nonzero(x))
# print(WHITE[0])
# f = y[2:5, 1:3]
# average = np.average(f)
# print(average)

print(x)