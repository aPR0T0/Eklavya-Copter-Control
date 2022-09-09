#! /usr/bin/env python3     #For a ROS Node makes sure the script is executed as a Python script
import rospy #for ROS operations
import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler #For Inter-conversions between Euler and Quaternion
import message_filters

# For all ros_msgs/ros_topics
from std_msgs.msg import Float64MultiArray,Float64 # Standard msg files for sending/receiving data, will be used for Tuning from rqt
from sensor_msgs.msg import Imu # Msg for Imu - msg file has variables defined which /omav/imu will contain
from nav_msgs.msg import Odometry # Msg for Odometry - msg file has variables defined which /omav/odometry_sensor1/odometry will contain
from mav_msgs.msg import Actuators # Msg for Actuators - msg file has variables defined which /omav/command/motor_speed will contain
from rosgraph_msgs.msg import Clock # Msg for Clock - msg file has variables defined which /clock will contain

#Call functions in other files
from utilities import *
from moment_desired import * 



# DECLARING / INITIALIZING ALL PARAMETERS

# Flag for 1st Time Calculation Functions running, to prevent Garbage Values or Local Variable referenced before assignment error
flag = 0
# Current Time - which is a floating point number, its in seconds+nano-seconds format
current_time = 0
"""
Defining 3*3 Inertial Matrix - Defined in xacro file of model
Current we are only considering Ixx, Iyy & Izz as symmetric model(assumption and other values are negligible)
Inertial_Matrix = | Ixx  Ixy  Ixz |
                  | Iyx  Iyy  Iyz |
                  | Izx  Izy  Izz |
"""
Inertial_Matrix = np.array([[0.0075, 0, 0],[0, 0.010939, 0],[0, 0, 0.01369]])
# Position (Co-ordinates) Desired Matrix, which is a 3*1 Matrix
position_desired = np.zeros((3, 1))
# Position (Co-ordinates) Current Matrix, which is a 3*1 Matrix
position_current = np.zeros((3, 1))
"""
Offset of Sensor from Origin at launch, this is due to sensor placed at the COM(Centre-of-Mass) which is slightly higher than the ground, 
since the drone rests on its legs and the main-frame is slightly higher than the ground, for clearance from ground for rotors, so they don't touch ground.
This offset is for the sensor being at COM - is mostly in the z-axis(in height/altitude format) as it subtracted from readings.
This subtraction is done, so when the drone lands it's desired altitude will be 0, but due to this error, 
it will not land softly on the ground, but crash or have a rough landing, hence it is required to subtract this error.

Many Times sensors might not be placed at COM of Drone, for which the error, will cause the drone to reach wrong co-ordinate, 
and land at wrong co-ordinate, hence we require to subtract this array

It is an array of Length = 3
position_current_error = | Error_x  Error_y  Error_z |
Calculated/Found by echoing rostopic of sensor which supplies Position(Co-ordinates) with drone at Launch Position(stationary)
"""
position_current_error = np.array([0, 0, 0.1749999999801301])
# Orientation (Euler_Angles) Desired Array, which is an array of Length = 3
euler_desired = np.zeros(3)
# Orientation (Euler_Angles) Current Array, which is an array of Length = 3
euler_current = np.zeros(3)
# Quaternion Orientation Desired Array, which is an array of Length = 4
quaternion_desired = np.zeros(4)
# Quaternion Orientation Current Array, which is an array of Length = 4
quaternion_current = np.zeros(4)
# Current Angular Velocity Matrix of Drone, which is a 3*1 Matrix
w_current = np.zeros((3, 1))
# 3*1 Matrix of Moment_Desired
M_desired = np.zeros((3, 1))



def get_position_desired():
    """
    Calls Function, call_position_desired() from utilities.py which takes Desired Position - in Co-ordinates Format
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global position_desired

    position_desired = call_position_desired()

def get_orientation_desired():
    """
    Calls Function, call_orientation_desired() from utilities.py which takes Desired Orientation - in Euler Angles Format
    But for Calculations we required Desired Orientation - in Quaternion Format
    Hence it also calls function, euler_to_quaternion() from utilities.py which converts Euler Angles to Quaternion Orientation
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global euler_desired, quaternion_desired

    euler_desired = call_orientation_desired()
    # Since we require orientation in Quaternion format, hence converting euler-to-quaternion
    quaternion_desired = euler_to_quaternion(euler_desired)



def master(imu_subscriber, odometry_subscriber, clock_subscriber):
    """
    Master Function which makes calls to all functions, to get, process and publish data
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global flag, current_time, position_current, quaternion_current, euler_current
    global position_desired, quaternion_desired
    global M_desired, Inertial_Matrix

    # SENSOR READINGS FUNCTION CALLS
    current_time = get_time(clock_subscriber)

    position_current = get_position_current(odometry_subscriber, position_current_error)

    quaternion_current = get_orientation_current(imu_subscriber)

    # Since for a few calculations we need Current Orientation in Euler Angles format
    euler_current = quaternion_to_euler(quaternion_current)

    w_current = get_current_angular_velocity(imu_subscriber)



    # Moment Desired Calculations Function Call
    M_desired = moment_desired(quaternion_desired, quaternion_current, w_current, Inertial_Matrix, kq, kr, flag, r_offset, F_desired)

    flag+=1



def control():
    """
    This is the main control function called by main
    It initializes node
    Subscribes to Topics
    """
    #Initializing Node
    rospy.init_node('Omav_Controller_Node', anonymous=False)

    # Subscribers to get all relevant sensor readings - Multiple Sensors are used for redundancy
    imu_subscriber = message_filters.Subscriber("/omav/imu", Imu) # For Quaternion Orientation, Angular Velocity & Linear Acceleration(Not Currently)
    odometry_subscriber = message_filters.Subscriber("/omav/odometry_sensor1/odometry", Odometry) # For Position, Linear Velocity(Not Currently) & Angular Acceleration(Not Currently)
    clock_subscriber = message_filters.Subscriber("/clock", Clock) # For Time
    # To time-sync both the subscribers and only, to use data when both publishers subscribe at the same time, this method is used
    ts = message_filters.TimeSynchronizer([imu_subscriber,odometry_subscriber, clock_subscriber], 10)

    # register multiple callbacks with this method, which will get called in the order they are registered
    ts.registerCallback(master)
    
    rospy.spin() # For code to run in a loop several times
    # Please note, any code written after this will not run, it will only run when ROS is Interrupted/Closed



if __name__=='__main__':
    try:
        get_position_desired()
        get_orientation_desired()
        control()
    except rospy.ROSInterruptException:
        pass