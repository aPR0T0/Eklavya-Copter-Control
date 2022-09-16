import numpy as np
import math

a1 = np.zeros((3, 3))

print("Zero Matrix : \n", a1)

a1[0][0] = 1
a1[1][0] = 2
a1[2][0] = 3

a1[0][1] = 4
a1[1][1] = 5
a1[2][1] = 6


a1[0][2] = 7
a1[1][2] = 8
a1[2][2] = 9


print("Edited Matrix \n", a1)
print(type(a1))