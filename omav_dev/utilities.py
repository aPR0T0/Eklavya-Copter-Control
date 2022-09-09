import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Declaring Variables
roll = 0
pitch = 0
yaw = 0
quaternion_returned = np.zeros(4)
euler_returned = np.zeros(3)
required_position_returned = np.zeros(3)
required_orientation_returned = np.zeros(3)



# INPUT FROM USER - Functions
def call_position_desired():
    """
    Taking Position Desired Input from User
    Taken in X, Y and Altitude(Z) Format of Co-ordinates
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global required_position_returned
    
    # Taking Input from User
    required_position_returned[0], required_position_returned[2], required_position_returned[2] = map(float, input("Enter Desired X Y (Position) and Altitude Co-ordinates : ").split())
    
    return(required_position_returned)


def call_orientation_desired():
    """
    Taking Orientation Desired Input from User
    Taken in Roll, Pitch and Yaw Format of Euler Angles - Angles in Degrees
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global required_orientation_returned

    # Taking Input from User
    required_orientation_returned[0], required_orientation_returned[1], required_orientation_returned[2] = map(float, input("Enter Desired Orientation Roll, Pitch and Yaw - Euler Angles in Degrees : ").split())

    return(required_orientation_returned)



# CONVERSION Functions
def euler_to_quaternion(euler_supplied):
    """
    Convert Euler Angles - Roll, Pitch & Yaw
    To Corresponding Quaternion Orientation - X, Y, Z & W terms
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global roll, pitch, yaw, quaternion_returned

    # Since we are supplying angles in degrees, but for actual calculations we need angles in radians
    roll = euler_supplied[0] * (math.pi/180)
    pitch = euler_supplied[1] * (math.pi/180)
    yaw = euler_supplied[2] * (math.pi/180)

    quaternion_returned = quaternion_from_euler(roll,pitch,yaw)

    return(quaternion_returned)


def quaternion_to_euler(quaternion_supplied):
    """
    Convert Quaternion Orientation - X, Y, Z & W
    To Corresponding Euler Angles - Roll, Pitch & Yaw terms
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global euler_returned

    euler_returned = euler_from_quaternion(quaternion_supplied)

    euler_returned = np.array(euler_returned)
    # Since for calculations we require angles in radians, hence we retain them in radians, rather than converting them to degrees
    return(euler_returned)