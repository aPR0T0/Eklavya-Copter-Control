import numpy as np
import math

q_v_error1 = np.matrix([[1], [2], [3]])
q_v_error2 = np.matrix([[3], [2], [1]])

q = q_v_error1 - q_v_error2

print(q)
print(q[1])