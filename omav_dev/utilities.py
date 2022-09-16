#! /usr/bin/env python3
import rospy
import message_filters
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray,Float64



# Declaring Variables

# Desired Position Returned which is a 3*1 Matrix
desired_position_returned = np.zeros((3, 1))
# Desired Orientation Returned(Euler Angles Format) which is an array of Length = 3
desired_orientation_returned = np.zeros(3)

# Roll, Pitch & Yaw for calculations during conversions
roll_conversion = 0
pitch_conversion = 0
yaw_conversion = 0
# Quaternion Orientation Returned which is an array of Length = 4
quaternion_returned = np.zeros(4)
# Euler Angles Returned which is an array of Length = 3
euler_returned = np.zeros(3)

# Getting Time from Clock, and calculating/converting it to required format
secs = 0
nsecs = 0
time_returned = 0
# Current Position Returned which is a 3*1 Matrix
current_position_returned = np.zeros((3, 1))
# Current Orientation Returned(Quaternion Orientation Format) which is an array of Length = 4
current_orientation_returned = np.zeros(4)
# Current Angular Velocity Returned which is a 3*1 Matrix
current_angular_velocity_returned = np.zeros((3, 1))

#
position_current_error = np.zeros(3)



def set_position_current_error(set_position_current_error):
    """
    Sets's position_current_error to be used by get_position_current
    """
    #
    global position_current_error
    position_current_error = set_position_current_error
    #print("Position Current Error printed from utilities.py :", position_current_error)
    #print(type(position_current_error))


# INPUT FROM USER - Functions
def call_position_desired():
    """
    Taking Position Desired Input from User
    Taken in X, Y and Altitude(Z) Format of Co-ordinates
    Returned as a 3*1 Matrix since we require that format for Calculations
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global desired_position_returned

    # Taking Input from User
    desired_position_returned[0, 0], desired_position_returned[1, 0], desired_position_returned[2, 0] = map(float, input("Enter Desired X Y (Position) and Altitude Co-ordinates : ").split())

    #print("Position_Desired printed from utilities.py :", desired_position_returned)
    #print(type(desired_position_returned))

    return(desired_position_returned)


def call_orientation_desired():
    """
    Taking Orientation Desired Input from User
    Taken in Roll, Pitch and Yaw Format of Euler Angles - Angles in Degrees
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global desired_orientation_returned

    # Taking Input from User
    desired_orientation_returned[0], desired_orientation_returned[1], desired_orientation_returned[2] = map(float, input("Enter Desired Orientation Roll, Pitch and Yaw - Euler Angles in Degrees : ").split())

    #print("Orientation_Desired in Euler_Angles(degrees) printed from utilities.py :", desired_orientation_returned)
    #print(type(desired_orientation_returned))

    return(desired_orientation_returned)



# CONVERSION Functions
def euler_to_quaternion(euler_supplied):
    """
    Convert Euler Angles - Roll, Pitch & Yaw
    To Corresponding Quaternion Orientation - X, Y, Z & W terms
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global roll_conversion, pitch_conversion, yaw_conversion, quaternion_returned

    # Since we are supplying angles in degrees, but for actual calculations we need angles in radians
    roll_conversion = euler_supplied[0] * (math.pi/180)
    pitch_conversion = euler_supplied[1] * (math.pi/180)
    yaw_conversion = euler_supplied[2] * (math.pi/180)
    
    #print("Orientation in Euler_Angles(radians) printed from utilities.py :", roll_conversion, pitch_conversion, yaw_conversion)
    #print(type(roll_conversion), type(pitch_conversion), type(yaw_conversion))

    quaternion_returned = quaternion_from_euler(roll_conversion, pitch_conversion, yaw_conversion)

    #print("Orientation in Quaternion printed from utilities.py :", quaternion_returned)
    #print(type(quaternion_returned))

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
    
    #print("Orientation in Euler Angles(radians) printed from utilities.py :", euler_returned)
    #print(type(euler_from_quaternion))

    return(euler_returned)



# GET DATA FROM SENSORS
def get_time(msg):
    """
    Get Current Time Readings from Clock
    Clock publishes readings in Seconds and Nano-seconds format, which we convert to single term per our requirement
    Time is used in the Derivative Term of the PID Controller which calculates the F_desired term to be used in Control Allocation Equation
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global secs, nsecs, time_returned

    # Since we get values from sensor in secs and nsecs format, and we require it as a single term
    secs = msg.header.stamp.secs
    nsecs = (msg.header.stamp.nsecs)
    time_returned = secs + (nsecs/1000000000)

    #print("Time in secs & nsecs printed in utilities.py :", secs, nsecs)
    #print(type(secs), type(nsecs))
    #print("Current Time printed from utilites.py :", time_returned)
    #print(type(time_returned))

    return(time_returned)


def get_position_current(msg):
    """
    Get Current Position(Co-ordinates) of Drone from Sensor - Odometry
    Taken in X, Y and Altitude(Z) Format of Co-ordinates
    Also Initial Launch Error is subtracted from Current Position
    Returned as a 3*1 Matrix since we require that format for Calculations
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global current_position_returned, position_current_error

    # Subtracting Intial Launch Error from Current Position
    current_position_returned[0, 0] = (msg.pose.pose.position.x - position_current_error[0])
    current_position_returned[1, 0] = (msg.pose.pose.position.y - position_current_error[1])
    current_position_returned[2, 0] = (msg.pose.pose.position.z - position_current_error[2])

    #print("Current Position printed from utilities.py : \n", current_position_returned)
    #print(type(current_position_returned))

    return(current_position_returned)


def get_orientation_current(msg):
    """
    Getting Current Orientation(Quaternion Format) of Drone from Sensor - Imu
    Taken in Quaternion Orientation - X, Y, Z & W Format
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global current_orientation_returned

    current_orientation_returned[0] = msg.orientation.x
    current_orientation_returned[1] = msg.orientation.y
    current_orientation_returned[2] = msg.orientation.z
    current_orientation_returned[3] = msg.orientation.w

    #print("Current Quaternion Orientation printed from utilities.py :", current_orientation_returned)
    #print(type(current_orientation_returned))

    return(current_orientation_returned)


def get_current_angular_velocity(msg):
    """
    Getting Current Angular Velocity of Drone from Sensor - Imu
    Taken in X, Y & Z Format
    Returned as a 3*1 Matrix since we require that format for Calculations
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global current_angular_velocity_returned

    current_angular_velocity_returned[0, 0] = msg.angular_velocity.x
    current_angular_velocity_returned[1, 0] = msg.angular_velocity.y
    current_angular_velocity_returned[2, 0] = msg.angular_velocity.z

    #print("Current Angular Velocity printed from utilities.py : \n", current_angular_velocity_returned)
    #print(type(current_angular_velocity_returned))

    return(current_angular_velocity_returned)