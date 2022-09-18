#! /usr/bin/env python3     #For a ROS Node makes sure the script is executed as a Python script
from turtle import position
import rospy #for ROS operations
import numpy as np
import math
import statistics
import time
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



"""
VALUES DEFINED IN MODEL, WHICH REQUIRES TO BE INITIALIZED

Inertial_Matrix :
Defining 3*3 Inertial Matrix - Defined in xacro file of model, omav.xacro
Current we are only considering Ixx, Iyy & Izz as symmetric model(assumption and other values are negligible, due to which other values equated to 0)
Inertial_Matrix = | Ixx  Ixy  Ixz |
                  | Iyx  Iyy  Iyz |
                  | Izx  Izy  Izz |

gravity:
Gravity Matrix, which is a 3*1 Matrix - Defined in xacro file of model, omav.xacro
Since, on Earth Gravity only in z-direction, hence rest values are 0
gravity = | g_x |
          | g_y |
          | g_z |

mass :
Mass of Drone at COM(Centre of Mass) in kg - Defined in xacro file of model, omav.xacro
It is a scalar

arm_length :
Arm Length of Drone in m from COM to thrust-providing Rotor - Defined in xacro file of model, omav.xacro
It is a scalar

r_offset :
Center of Mass Offset Matrix, which is a 3*1 Matrix - Defined in xacro file of model, omav.xacro

position_current_error :
Offset of Sensor from Origin at launch, this is due to sensor placed at the COM(Centre-of-Mass) which is slightly higher than the ground, since the drone rests on its legs and the main-frame is slightly higher than the ground, for clearance from ground for rotors, so they don't touch ground.
This offset is for the sensor being at COM - is mostly in the z-axis(in height/altitude format) as it subtracted from readings.
This subtraction is done, so when the drone lands it's desired altitude will be 0, but due to this error, it will not land softly on the ground, but crash or have a rough landing, hence it is required to subtract this error.
Many Times sensors might not be placed at COM of Drone, for which the error, will cause the drone to reach wrong co-ordinate, 
and land at wrong co-ordinate, hence we require to subtract this array
It is an array of Length = 3
position_current_error = | Error_x  Error_y  Error_z |
Calculated/Found by echoing rostopic of sensor which supplies Position(Co-ordinates) with drone at Launch Position(stationary)
"""
Inertial_Matrix = np.array([[0.0075, 0, 0], [0, 0.010939, 0], [0, 0, 0.01369]])
gravity = np.array([[0], [0], [-9.80]])
mass = 4.04
arm_length = 0.300
r_offset = np.array([[0], [0], [0]])
position_current_error = np.array([0, 0, 0.1749999999801301])



"""
DECLARING / INITIALIZING ALL PARAMETERS

flag : Flag for 1st Time Calculation Functions running, to prevent Garbage Values or Local Variable referenced before assignment error
current_time : which is a floating point number, its current time in seconds+nano-seconds format
sample_time : Minimum Sample Size for Time
position_desired : Position (Co-ordinates) Desired Matrix, which is a 3*1 Matrix
position_current : Position (Co-ordinates) Current Matrix, which is a 3*1 Matrix
euler_desired : Orientation (Euler_Angles) Desired Array, which is an array of Length = 3
euler_current : Orientation (Euler_Angles) Current Array, which is an array of Length = 3
quaternion_desired : Quaternion Orientation Desired Array, which is an array of Length = 4
quaternion_current : Quaternion Orientation Current Array, which is an array of Length = 4
w_current : Current Angular Velocity Matrix of Drone, which is a 3*1 Matrix
kp : Proportional Gain of PID Controller of F_desired Calculation
kd : Derivative Gain of PID Controller of F_desired Calculation
ki : Integral Gain of PID Controller of F_desired Calculation
kq : Tuning Parameter of M_desired Calculation
kr : Rate Controller Gain of M_desired Calculation
Mu : Lift Force Coefficient
kappa : Drag Torque Coefficient
F_desired : Force Desired Matrix, which is a 3*1 Matrix
M_desired : Moment Desired Matrix, which is a 3*1 Matrix
F_dec : Force dec Matrix, which is a 6*1 Matrix
"""

flag = 0
current_time = 0
sample_time = 0.005
position_desired = np.zeros((3, 1))
position_current = np.zeros((3, 1))
euler_desired = np.zeros(3)
euler_current = np.zeros(3)
quaternion_desired = np.zeros(4)
quaternion_current = np.zeros(4)
w_current = np.zeros((3, 1))
kp = 50
kd = 0
ki = 0
kq = 0.00005
kr = 0.0000005
Mu = 7.2e-06
kappa = 8.06428e-05
F_desired = np.zeros((3, 1))
M_desired = np.zeros((3, 1))
F_dec = np.zeros((24, 1))

"""

"""
#prev_time = 0
#average_time = 0
#prev_error = 0
#error = 0
#a1 = []
# Creating Publisher to publish rotor_speeds and tilt-rotor angles



speed_pub = rospy.Publisher("/omav/command/motor_speed", Actuators ,queue_size=10)



# GAINS FUNCTIONS
def set_proportional_gain(msg):
    global kp
    kp = msg.data
    #print("Kp from controller.py :", kp)
    #print(type(kp))

def set_derivative_gain(msg):
    global kd
    kd = msg.data
    #print("Kd from controller.py :", kd)
    #print(type(kp))

def set_integral_gain(msg):
    global ki
    ki = msg.data
    #print("Ki from controller.py :", ki)
    #print(type(kp))

def set_tuning_parameter(msg):
    global kq
    kq = msg.data
    #print("Kq from controller.py :", kq)
    #print(type(kp))

def set_rate_controller_gain(msg):
    global kr
    kr = msg.data
    #print("Kr from controller.py :", kr)
    #print(type(kp))

"""
def set_lift_force_coefficient(msg):
    global Mu
    Mu = msg.data
    #print("Mu from controller.py :", Mu)
    #print(type(kp))

def set_drag_torque_coefficient(msg):
    global kappa
    kappa = msg.data
    #print("kappa from controller.py :", kappa)
    #print(type(kp))
"""

# Get_User_Input Functions, called only once by main
def get_position_desired():
    """
    Calls Function, call_position_desired() from utilities.py which takes Desired Position - in Co-ordinates Format
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global position_desired

    position_desired = call_position_desired()
    
    #print("Position_Desired printed from controller.py :\n", position_desired)
    #print(type(position_desired))


def get_orientation_desired():
    """
    Calls Function, call_orientation_desired() from utilities.py which takes Desired Orientation - in Euler Angles Format
    But for Calculations we required Desired Orientation - in Quaternion Format
    Hence it also calls function, euler_to_quaternion() from utilities.py which converts Euler Angles to Quaternion Orientation
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global euler_desired, quaternion_desired

    euler_desired = call_orientation_desired()
    #print("Orientation_Desired in Euler_Angles(degrees) printed from controller.py :", euler_desired)
    #print(type(euler_desired))

    # Since we require orientation in Quaternion format, hence converting euler-to-quaternion
    quaternion_desired = euler_to_quaternion(euler_desired)
    #print("Orientation_Desired in Quaternion printed from controller.py :", quaternion_desired)
    #print(type(quaternion_desired))



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
    #global average_time, prev_time, error, prev_error, a1


    # SENSOR READINGS FUNCTION CALLS
    #current_time = get_time(odometry_subscriber)
    current_time = time.time()
    """
    if (flag == 0):
        print(flag)
        #prev_time = current_time
        #print(1)

    if (flag != 0):
        #error = current_time - prev_time
        #a1.append(error)
        #average_time = statistics.mean(a1)
        #prev_time = current_time
        #print("Current Time printed from controller.py :", average_time)
        #print(type(average_time))
        print(flag)
    """
    print("Kp :", kp)

    set_position_current_error(position_current_error)
    position_current = get_position_current(odometry_subscriber)
    #print("Current Position printed from controller.py : \n", position_current)
    #print(type(position_current))
    
    #print(position_current[2][0])
    altitude = float(round(position_current[2][0], 2))
    #print(altitude)
    #print(type(altitude))
    alt_err_pub = rospy.Publisher("/altitude_err", Float64, queue_size=10)
    alt_err_pub.publish(altitude)
    

    quaternion_current = get_orientation_current(imu_subscriber)
    #print("Current Quaternion Orientation printed from controller.py :",quaternion_current)
    #print(type(quaternion_current))

    # Since for a few calculations we need Current Orientation in Euler Angles format
    euler_current = quaternion_to_euler(quaternion_current)
    #print("Orientation in Euler Angles(radians) printed from controller.py :", euler_current)
    #print(type(euler_current))

    w_current = get_current_angular_velocity(imu_subscriber)
    #print("Current Angular Velocity printed from controller.py : \n", w_current)
    #print(type(w_current))


    # GAINS Subscribers
    rospy.Subscriber("Proportional_Gain", Float64, set_proportional_gain)
    rospy.Subscriber("Derivative_Gain", Float64, set_derivative_gain)
    rospy.Subscriber("Integral_Gain", Float64, set_integral_gain)
    rospy.Subscriber("Tuning_Parameter", Float64, set_tuning_parameter)
    rospy.Subscriber("Rate_Controller_Gain", Float64, set_rate_controller_gain)
    #rospy.Subscriber("Lift_Force_Coefficient", Float64, set_lift_force_coefficient)
    #rospy.Subscriber("Drag_Torque_Coefficient", Float64, set_drag_torque_coefficient)

    
    # Force Desired Calculations Function Call
    F_desired = force_desired(position_desired, position_current, euler_current, current_time, sample_time, kp, kd, ki, mass, gravity, flag)
    #print("1, \n", F_desired)

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
    #print("Flag printed from controller.py :", flag)
    #print(type(flag))

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