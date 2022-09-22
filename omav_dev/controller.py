#! /usr/bin/env python3     #For a ROS Node makes sure the script is executed as a Python script

import rospy #for ROS operations
import numpy as np
import math

import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler #For Inter-conversions between Euler and Quaternion
import message_filters

# For all ros_msgs/ros_topics
from std_msgs.msg import Float64MultiArray,Float64 # Standard msg files for sending/receiving data, will be used for Tuning from rqt
from sensor_msgs.msg import Imu # Msg for Imu - msg file has variables defined which /omav/imu will contain
from nav_msgs.msg import Odometry # Msg for Odometry - msg file has variables defined which /omav/odometry_sensor1/odometry will contain
from mav_msgs.msg import Actuators # Msg for Actuators - msg file has variables defined which /omav/command/motor_speed will contain

#Call functions in other files
from force_desired import *
from moment_desired import *
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

startup_position_error :
Offset of Sensor from Origin at launch, this is due to sensor placed at the COM(Centre-of-Mass) which is slightly higher than the ground, since the drone rests on its legs and the main-frame is slightly higher than the ground, for clearance from ground for rotors, so they don't touch ground.
This offset is for the sensor being at COM - is mostly in the z-axis(in height/altitude format) as it subtracted from readings.
This subtraction is done, so when the drone lands it's desired altitude will be 0, but due to this error, it will not land softly on the ground, but crash or have a rough landing, hence it is required to subtract this error.
Many Times sensors might not be placed at COM of Drone, for which the error, will cause the drone to reach wrong co-ordinate, 
and land at wrong co-ordinate, hence we require to subtract this array
It is an array of Length = 3
startup_position_error = | Error_x  Error_y  Error_z |
Calculated/Found by echoing rostopic of sensor which supplies Position(Co-ordinates) with drone at Launch Position(stationary)
"""
Inertial_Matrix = np.array([[0.0075, 0, 0], [0, 0.010939, 0], [0, 0, 0.01369]])
gravity = np.array([[0], [0], [-9.81]])
mass = 4.04
arm_length = 0.300
r_offset = np.array([[0], [0], [0]])
startup_position_error = np.array([0, 0, 0.1749999999801301])

speed_publisher = Actuators()



"""
DECLARING / INITIALIZING ALL PARAMETERS

flag : Flag for 1st Time Calculation Functions running, to prevent Garbage Values or Local Variable referenced before assignment error
current_time : which is a floating point number, its current time in seconds+nano-seconds format
sample_time : Minimum Sample Size for Time
position_desired : Position (Co-ordinates) Desired Matrix, which is a 3*1 Matrix
position_current : Position (Co-ordinates) Current Matrix, which is a 3*1 Matrix
current_position_returned : Position (Co-ordinates) Current Matrix, which is a 3*1 Matrix, returned from Odometry Sensor
euler_desired : Orientation (Euler_Angles) Desired Array, which is an array of Length = 3
euler_current : Orientation (Euler_Angles) Current Array, which is an array of Length = 3
euler_returned : Orientation (Euler_Angles) Current Array, which is an array of Length = 3, converted from IMU Sensor Quaternion to Euler
quaternion_desired : Quaternion Orientation Desired Array, which is an array of Length = 4
quaternion_current : Quaternion Orientation Current Array, which is an array of Length = 4
current_orientation_returned : Quaternion Orientation Current Array, which is an array of Length = 4, returned from IMU Sensor
w_current : Current Angular Velocity Matrix of Drone, which is a 3*1 Matrix
current_angular_velocity_returned : Current Angular Velocity Matrix of Drone, which is a 3*1 Matrix, returned from IMU Sensor
kp : Proportional Gain (individually for x, y and z) of PID Controller of F_desired Calculation, which is a 1*3 Matrix
kd : Derivative Gain (individually for x, y and z) of PID Controller of F_desired Calculation, which is a 1*3 Matrix
ki : Integral Gain (individually for x, y and z) of PID Controller of F_desired Calculation, which is a 1*3 Matrix
kq : Tuning Parameter of M_desired Calculation
kr : Rate Controller Gain of M_desired Calculation
Mu : Lift Force Coefficient
kappa : Drag Torque Coefficient
F_desired : Force Desired Matrix, which is a 3*1 Matrix
M_desired : Moment Desired Matrix, which is a 3*1 Matrix
F_dec : Force dec Matrix, which is a 12*1 Matrix
current_position_error : Current Position (Co-ordinates) Error Matrix, which is a 3*1 Matrix
"""

flag = 0
current_time = 0
sample_time = 0.005
position_desired = np.zeros((3, 1))
position_current = np.zeros((3, 1))
current_position_returned = np.zeros((3, 1))
euler_desired = np.zeros(3)
euler_current = np.zeros(3)
euler_returned = np.zeros(3)
quaternion_desired = np.zeros(4)
quaternion_current = np.zeros(4)
current_orientation_returned = np.zeros(4)
w_current = np.zeros((3, 1))
current_angular_velocity_returned = np.zeros((3, 1))
kp = np.array([0, 0, 5])
kd = np.zeros(3)
ki = np.zeros(3)
kq = 100
kr = 4
Mu = 7.2e-06
kappa = 8.06428e-05
F_desired = np.zeros((3, 1))
M_desired = np.zeros((3, 1))
F_dec = np.zeros((12, 1))
current_position_error = np.zeros((3, 1))
splitting_constant = round(math.sqrt(2), 2)
# F_dec = np.zeros((24, 1))



# Creating Publisher to publish rotor_speeds and tilt-rotor angles
speed_pub = rospy.Publisher("/omav/command/motor_speed", Actuators ,queue_size=100)



# Get_User_Input Functions, called only once by main
"""
Taking Position Desired Input from User
Taken in X, Y and Altitude(Z) Format of Co-ordinates
Returned as a 3*1 Matrix since we require that format for Calculations
"""


position_desired[0][0], position_desired[1][0], position_desired[2][0] = map(float, input("Enter Desired X Y (Position) and Altitude Co-ordinates : ").split())



"""
Taking Orientation Desired Input from User
Taken in Roll, Pitch and Yaw Format of Euler Angles - Angles in Degrees
But for Calculations we required Desired Orientation - in Quaternion Format
Hence converted Euler Angles to Quaternion Orientation
"""

euler_desired[0], euler_desired[1], euler_desired[2] = map(float, input("Enter Desired Orientation Roll, Pitch and Yaw - Euler Angles in Degrees : ").split())

# Since we require orientation in Quaternion format, hence converting euler-to-quaternion
# Since we are supplying angles in degrees, but for actual calculations we need angles in radians
for i in range (0, 3):
    euler_desired[i] = euler_desired[i] * (math.pi/180)

quaternion_desired = quaternion_from_euler(euler_desired[0], euler_desired[1], euler_desired[2])



# Current Sensor Readings
def get_position_current(msg):
    """
    Get Current Position(Co-ordinates) of Drone from Sensor - Odometry
    Taken in X, Y and Altitude(Z) Format of Co-ordinates
    Also Initial Launch Error is subtracted from Current Position
    Returned as a 3*1 Matrix since we require that format for Calculations
    """
    global current_position_returned, startup_position_error

    # Subtracting Intial Launch Error from Current Position
    current_position_returned[0][0] = (msg.pose.pose.position.x - startup_position_error[0])
    current_position_returned[1][0] = (msg.pose.pose.position.y - startup_position_error[1])
    current_position_returned[2][0] = (msg.pose.pose.position.z - startup_position_error[2])

    return(np.round_(current_position_returned, decimals=2))


def get_orientation_current(msg):
    """
    Getting Current Orientation(Quaternion Format) of Drone from Sensor - Imu
    Taken in Quaternion Orientation - X, Y, Z & W Format
    """
    global current_orientation_returned, euler_returned

    current_orientation_returned[0] = msg.orientation.x
    current_orientation_returned[1] = msg.orientation.y
    current_orientation_returned[2] = msg.orientation.z
    current_orientation_returned[3] = msg.orientation.w

    euler_returned = euler_from_quaternion(current_orientation_returned)
    euler_returned = np.array(euler_returned)

    return(np.round_(current_orientation_returned, decimals=2), np.round_(euler_returned, decimals=2))


def get_current_angular_velocity(msg):
    """
    Getting Current Angular Velocity of Drone from Sensor - Imu
    Taken in X, Y & Z Format
    Returned as a 3*1 Matrix since we require that format for Calculations
    """
    global current_angular_velocity_returned

    current_angular_velocity_returned[0][0] = msg.twist.twist.angular.x
    current_angular_velocity_returned[1][0] = msg.twist.twist.angular.y
    current_angular_velocity_returned[2][0] = msg.twist.twist.angular.z

    return(np.round_(current_angular_velocity_returned, decimals=2))



# GAINS FUNCTIONS
def set_proportional_gain(msg):
    global kp
    kp = np.array([msg.data[0], msg.data[1], msg.data[2]])

def set_derivative_gain(msg):
    global kd
    kd = np.array([msg.data[0], msg.data[1], msg.data[2]])

def set_integral_gain(msg):
    global ki
    ki = np.array([msg.data[0], msg.data[1], msg.data[2]])


def set_tuning_parameter(msg):
    global kq
    kq = msg.data

def set_rate_controller_gain(msg):
    global kr
    kr = msg.data



# MASTER CALLING FUNCTION
def master(imu_subscriber, odometry_subscriber):
    """
    Master Function which makes calls to all functions, to get, process and publish data
    """
    # To prevent Garbage Values being used or variables being initialized/reset as zero
    global flag, current_time, sample_time, position_current, quaternion_current, euler_current, w_current, r_offset
    global position_desired, quaternion_desired, F_desired, M_desired, F_dec, Inertial_Matrix, gravity, mass, arm_length
    global kp, kd, ki, kq, kr, Mu, kappa, splitting_constant


    current_time = time.time()

    # SENSOR READINGS FUNCTION CALLS
    position_current = get_position_current(odometry_subscriber)

    quaternion_current, euler_current = get_orientation_current(imu_subscriber)
    # Since for a few calculations we need Current Orientation in Euler Angles format

    speed_publisher = Actuators()
    w_current = get_current_angular_velocity(odometry_subscriber)



    # GAINS Subscribers
    rospy.Subscriber("Proportional_Gain", Float64MultiArray, set_proportional_gain)
    rospy.Subscriber("Derivative_Gain", Float64MultiArray, set_derivative_gain)
    rospy.Subscriber("Integral_Gain", Float64MultiArray, set_integral_gain)
    rospy.Subscriber("Tuning_Parameter", Float64, set_tuning_parameter)
    rospy.Subscriber("Rate_Controller_Gain", Float64, set_rate_controller_gain)


    # Force Desired Calculations Function Call
    F_desired = force_desired(position_desired, position_current, euler_current, current_time, sample_time, kp, kd, ki, mass, gravity, flag)
    current_position_error = position_error()

    print(F_desired)
    # Moment Desired Calculations Function Call
    M_desired = moment_desired(quaternion_desired, quaternion_current, w_current, Inertial_Matrix, kq, kr, flag, r_offset, F_desired)


    # F_dec Calculations Function Call
    F_dec = force_dec(F_desired, M_desired, Mu, kappa, arm_length, flag)


    # speed_publisher Calculations Function Call
    speed_publisher = get_speed_publisher(F_dec, Mu, flag, splitting_constant)

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
    imu_subscriber = message_filters.Subscriber("/omav/ground_truth/imu", Imu) # For Quaternion Orientation, Angular Velocity & Linear Acceleration(Not Currently)
    odometry_subscriber = message_filters.Subscriber("/omav/ground_truth/odometry", Odometry) # For Position, Time, Linear Velocity(Not Currently) & Angular Acceleration(Not Currently)
    
    # To time-sync both the subscribers and only, to use data when both publishers subscribe at the same time, this method is used
    ts = message_filters.TimeSynchronizer([imu_subscriber,odometry_subscriber], 10)

    # register multiple callbacks with this method, which will get called in the order they are registered
    ts.registerCallback(master)
    
    rospy.spin() # For code to run in a loop several times
    # Please note, any code written after this will not run, it will only run when ROS is Interrupted/Closed



if __name__=='__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass