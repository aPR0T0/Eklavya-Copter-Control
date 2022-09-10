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

#Call functions in other files
from utilities import *
from moment_desired import * 
from force_desired import *
from force_dec import *
from speed_publisher import *



# DECLARING / INITIALIZING ALL PARAMETERS

# Flag for 1st Time Calculation Functions running, to prevent Garbage Values or Local Variable referenced before assignment error
flag = 0
# Current Time - which is a floating point number, its in seconds+nano-seconds format
current_time = 0
# Minimum Sample Size for Time
sample_time = 0.005
"""
Defining 3*3 Inertial Matrix - Defined in xacro file of model, omav.xacro
Current we are only considering Ixx, Iyy & Izz as symmetric model(assumption and other values are negligible, due to which other values equated to 0)
Inertial_Matrix = | Ixx  Ixy  Ixz |
                  | Iyx  Iyy  Iyz |
                  | Izx  Izy  Izz |
"""
Inertial_Matrix = np.array([[0.0075, 0, 0], [0, 0.010939, 0], [0, 0, 0.01369]])
"""
Gravity Matrix, which is a 3*1 Matrix - Defined in xacro file of model, omav.xacro
Since, on Earth Gravity only in z-direction, hence rest values are 0
gravity = | g_x |
          | g_y |
          | g_z |
"""
gravity = np.array([[0], [0], [-9.80]])
"""
Mass of Drone at COM(Centre of Mass) in kg - Defined in xacro file of model, omav.xacro
It is a scalar
"""
mass = 4.04
"""
Arm Length of Drone in m from COM to thrust-providing Rotor - Defined in xacro file of model, omav.xacro
It is a scalar
"""
arm_length = 0.300
"""
Center of Mass Offset Matrix, which is a 3*1 Matrix - Defined in xacro file of model, omav.xacro
"""
r_offset = np.array([[0], [0], [0]])
# Position (Co-ordinates) Desired Matrix, which is a 3*1 Matrix
position_desired = np.zeros((3, 1))
# Position (Co-ordinates) Current Matrix, which is a 3*1 Matrix
position_current = np.zeros((3, 1))
"""
Offset of Sensor from Origin at launch, this is due to sensor placed at the COM(Centre-of-Mass) which is slightly higher than the ground, since the drone rests on its legs and the main-frame is slightly higher than the ground, for clearance from ground for rotors, so they don't touch ground.
This offset is for the sensor being at COM - is mostly in the z-axis(in height/altitude format) as it subtracted from readings.
This subtraction is done, so when the drone lands it's desired altitude will be 0, but due to this error, it will not land softly on the ground, but crash or have a rough landing, hence it is required to subtract this error.
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
# Proportional Gain of PID Controller of F_desired Calculation
kp = 10
# Derivative Gain of PID Controller of F_desired Calculation
kd = 0
# Integral Gain of PID Controller of F_desired Calculation
ki = 0
# Tuning Parameter of M_desired Calculation
kq = 0.05
# Rate Controller Gain of M_desired Calculation
kr = 0.05
# Lift Force Coefficient
Mu = 0.0003
# Drag Torque Coefficient
kappa = 0.0003

#
F_desired = np.zeros((3, 1))
# 3*1 Matrix of Moment_Desired
M_desired = np.zeros((3, 1))
#
F_dec = np.zeros((12, 1))




# Creating Publisher to publish rotor_speeds and tilt-rotor angles
speed_pub = rospy.Publisher("/omav/command/motor_speed", Actuators ,queue_size=1000)



# GAINS FUNCTIONS
def set_proportional_gain(msg):
    global kp
    kp = msg.data

def set_derivative_gain(msg):
    global kd
    kd = msg.data

def set_integral_gain(msg):
    global ki
    ki = msg.data

def set_tuning_parameter(msg):
    global kq
    kq = msg.data

def set_rate_controller_gain(msg):
    global kr
    kr = msg.data

def set_lift_force_coefficient(msg):
    global Mu
    Mu = msg.data

def set_drag_torque_coefficient(msg):
    global kappa
    kappa = msg.data



def get_position_desired():
    """
    Calls Function, call_position_desired() from utilities.py which takes Desired Position - in Co-ordinates Format
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global position_desired

    position_desired = call_position_desired()
    #print(position_desired)

def get_orientation_desired():
    """
    Calls Function, call_orientation_desired() from utilities.py which takes Desired Orientation - in Euler Angles Format
    But for Calculations we required Desired Orientation - in Quaternion Format
    Hence it also calls function, euler_to_quaternion() from utilities.py which converts Euler Angles to Quaternion Orientation
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global euler_desired, quaternion_desired

    euler_desired = call_orientation_desired()
    #print(euler_desired)
    # Since we require orientation in Quaternion format, hence converting euler-to-quaternion
    quaternion_desired = euler_to_quaternion(euler_desired)
    #print(quaternion_desired)



# MASTER CALLING FUNCTION
def master(imu_subscriber, odometry_subscriber):
    """
    Master Function which makes calls to all functions, to get, process and publish data
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global flag, current_time, sample_time, position_current, quaternion_current, euler_current, position_current_error, w_current
    global position_desired, quaternion_desired, F_desired, F_dec
    global M_desired, Inertial_Matrix, gravity, mass, arm_length, r_offset
    global kp, kd, ki, kq, kr, Mu, kappa



    # SENSOR READINGS FUNCTION CALLS
    current_time = get_time(odometry_subscriber)
    #print(current_time)

    position_current = get_position_current(odometry_subscriber)
    #print(position_current)

    quaternion_current = get_orientation_current(imu_subscriber)
    #print(quaternion_current)

    # Since for a few calculations we need Current Orientation in Euler Angles format
    euler_current = quaternion_to_euler(quaternion_current)
    #print(euler_current)

    w_current = get_current_angular_velocity(imu_subscriber)
    #print(w_current)


    # GAINS Subscribers
    rospy.Subscriber("Proportional_Gain", Float64, set_proportional_gain)
    rospy.Subscriber("Derivative_Gain", Float64, set_derivative_gain)
    rospy.Subscriber("Integral_Gain", Float64, set_integral_gain)
    rospy.Subscriber("Tuning_Parameter", Float64, set_tuning_parameter)
    rospy.Subscriber("Rate_Controller_Gain", Float64, set_rate_controller_gain)
    rospy.Subscriber("Lift_Force_Coefficient", Float64, set_lift_force_coefficient)
    rospy.Subscriber("Drag_Torque_Coefficient", Float64, set_drag_torque_coefficient)

    
    # Force Desired Calculations Function Call
    F_desired = force_desired(position_desired, position_current, euler_current, current_time, sample_time, kp, kd, ki, mass, gravity, flag)
    #print(F_desired)

    # Moment Desired Calculations Function Call
    M_desired = moment_desired(quaternion_desired, quaternion_current, w_current, Inertial_Matrix, kq, kr, flag, r_offset, F_desired)
    #print(M_desired)

    # F_dec Calculations Function Call
    F_dec = force_dec(F_desired, M_desired, Mu, kappa, arm_length, flag)
    #print(F_dec)
    
    # speed_publisher Calculations Function Call
    speed_publisher = Actuators()
    speed_publisher = get_speed_publisher(F_dec, Mu, flag)
    #print(speed_publisher)
    

    flag+=1

    speed_pub.publish(speed_publisher)



# CONTROL FUNCTION - node & subscribers
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
    odometry_subscriber = message_filters.Subscriber("/omav/odometry_sensor1/odometry", Odometry) # For Position, Time, Linear Velocity(Not Currently) & Angular Acceleration(Not Currently)
    
    # To time-sync both the subscribers and only, to use data when both publishers subscribe at the same time, this method is used
    ts = message_filters.TimeSynchronizer([imu_subscriber,odometry_subscriber], 10)

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