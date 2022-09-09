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
#position_current = np.zeros((3, 1))
# Orientation (Euler_Angles) Desired Matrix, which is a 3*1 Matrix
euler_desired = np.zeros(3)
# Orientation (Euler_Angles) Current Matrix, which is a 3*1 Matrix
#euler_current = np.zeros(3)
# Quaternion Orientation Desired Matrix, which is a 4*1 Matrix
quaternion_desired = np.zeros(4)
# Quaternion Orientation Current Matrix, which is a 4*1 Matrix
#quaternion_current = np.zeros(4)

#w_current = np.zeros((3, 1))

# 3*1 Matrix of Moment_Desired
M_desired = np.zeros((3, 1))


def get_position_desired():
    """
    Calls Function, call_position_desired() from utilities.py
    """
    global position_desired
    position_desired = call_position_desired()
    


def get_orientation_desired():
    """
    Calls Function, call_orientation_desired() from utilities.py which takes Desired Orientation - in Euler Angles Format
    But for Calculations we required Desired Orientation - in Quaternion Format
    Hence it also calls function, euler_to_quaternion() from utilities.py which converts Euler Angles to Quaternion Orientation
    """
    global euler_desired, quaternion_desired
    euler_desired = call_orientation_desired()
    quaternion_desired = euler_to_quaternion(euler_desired)



def master(imu,odometry):


    global flag
    global M_desired, Inertial_Matrix



    M_desired = moment_desired(quaternion_desired, quaternion_current, w_current, Inertial_Matrix, kq, kr, flag, r_offset, F_desired)

    flag+=1


def control():
    """
    This is the main control function called by main
    It initializes node
    Subscribes to Topics

    """
    

    #Initializing Node
    rospy.init_node('controller_node', anonymous=False)

    # Subscribers to get all relevant sensor readings
    imu_subscriber = message_filters.Subscriber("/omav/ground_truth/imu", Imu) # For Quaternion Orientation & Angular Velocity
    odometry_subscriber = message_filters.Subscriber("/omav/ground_truth/odometry", Odometry) # For Position, Time & Linear Velocity
    tr = message_filters.TimeSynchronizer([imu_subscriber,odometry_subscriber], 10)

    # To time-sync both the subscribers and only, to use data when both publishers subscribe at the same time, this method is used
    tr.registerCallback(master)
    rospy.spin() # For code to run in a loop several times
    # Please note, any code written after this will not run, it will only run when ROS is Interrupted/Closed





if __name__=='__main__':
    try:
        get_position_desired()
        get_orientation_desired()
        control()
    except rospy.ROSInterruptException:
        pass