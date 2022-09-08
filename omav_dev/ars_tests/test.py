import numpy as np
import math

a1 = np.array([1],[1],[1])
print(a1)
print(type(a1))

a2 = np.array([2, 3, 4])

a3 = (1*2*a2)
print(a3)
print(type(a3))
print(a3[0])


q_v_error1 = np.matrix([[1], [2], [3]])
q_v_error2 = np.matrix([[3], [2], [1]])

q = q_v_error1 - q_v_error2

print(q)
print(q[1])