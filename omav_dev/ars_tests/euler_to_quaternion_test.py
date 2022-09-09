import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Declaring Variables
roll = 0
pitch = 0
yaw = 0
quaternion_supplied = np.zeros(4)
euler_returned = np.zeros(3)
print(type(euler_returned))
#def quaternion_to_euler(quaternion_supplied):
"""
Convert Quaternion Orientation - X, Y, Z & W
To Corresponding Euler Angles - Roll, Pitch & Yaw terms
"""

# To prevent Garbage Values being used or variables being initialized as zero
#global euler_returned

euler_returned = euler_from_quaternion([0, 0, 0, 1])

euler_returned = np.array(euler_returned)
# Since for calculations we require angles in radians, hence we retain them in radians, rather than converting them to degrees
print((euler_returned))