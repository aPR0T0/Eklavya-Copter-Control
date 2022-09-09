import re
import numpy as np
import math

required_position_returned = np.zeros(3)

required_position_returned[0], required_position_returned[2], required_position_returned[2] = map(float, input("Enter X Y (Position) and Altitude Co-ordinates : ").split())

print(required_position_returned)
print(type(required_position_returned))