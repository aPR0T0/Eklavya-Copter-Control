import numpy as np
import math

a1 = np.array([[1],[1],[1]])
print(a1)
print(type(a1))

a2 = np.array([[2],[3],[4]])

a3 = np.cross(a1, a2, axis=0)
print(a3)
print(type(a3))
print(a3[0])

b1 = a2[(0,0)]
print(b1)
print(type(b1))