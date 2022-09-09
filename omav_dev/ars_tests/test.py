import re
import numpy as np
import math

required_position_returned = np.zeros((3, 1))

required_position_returned[0, 0], required_position_returned[1, 0], required_position_returned[2, 0] = map(float, input("Enter X Y (Position) and Altitude Co-ordinates : ").split())

print(required_position_returned)
print(type(required_position_returned))