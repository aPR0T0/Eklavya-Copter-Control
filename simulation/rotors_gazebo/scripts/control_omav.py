#! /usr/bin/env python3
from pid_omav import *
from cmath import pi
import rospy #for basically any type of ros operations
import time #to get definite time for the dTime
import message_filters
from std_msgs.msg import Float64MultiArray,Float64
from nav_msgs.msg import Odometry #It contains overall type of readings getting from the Odometry sensor
from mav_msgs.msg import Actuators
from sensor_msgs.msg import Imu 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from dynamic_reconfigure.msg import Config
#from now on we will be call (roll pitch yaw) as 'RPY' in this module

# Initilization of all Parameters
altitude = 0.30999
thrust = 0
vel_x = 0 
vel_y = 0 
vel_z = 0
roll = 0
pitch = 0
yaw = 0
x = 0
y = 0
z  = 0
acc_x = 0
acc_y = 0
acc_z = 0
    
speed = Actuators()

speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
speed.angular_velocities.append(0)
# Flag for checking for the first time the script is run
flag = 0

# kp_thrust = 20
# ki_thrust = 0.001
# kd_thrust = 35
# kp_roll = 2
# ki_roll = 0.001
# kd_roll = 0.5
# kp_pitch = 2
# ki_pitch = 0.001
# kd_pitch = 0.5
# kp_yaw = 2
# ki_yaw = 0.001
# kd_yaw = 0.5
# kp_x = 0.13
# ki_x = 0.01
# kd_x =  0.003 #0.00015
# kp_y = 0.13
# ki_y = 0
# kd_y = 0.00015

kap = 0.0005 #> constant for the matrix

Mu = 0.0005  #> constant for the matrix

t1 = 0.866025404 #> sqrt(3)/2

len = 0.3 #> assuming that length is 0.3m 


#creating publisher for the speeds of the rotors
speed_pub = rospy.Publisher("/omav/command/motor_speed", Actuators ,queue_size=100) #here we will use angles section to give angles to the tilt rotors

# Asking user for the desired coordinates
target_x, target_y, req_alt = map( float , input("Enter X Y (position) and Altitude : ").split())
roll_desired, pitch_desired, yaw_desired = map( float , input("Enter desired orientation (in degrees) Roll, Pitch, Yaw (resp): ").split())
# We need x,y and altitude of the model
# split() : Return a list of the words in the string, using sep as the delimiter string
# map() : Basically provides the value recieved in it to the variables on the left with the given data type()


# Gets yaw PID published to node
def set_tuning_parameter(msg):
    global kq
    kq = msg.data

def set_rate_controller_gain(msg):
    global kr
    kr = msg.data

def setPID_x(msg):
    global kp_x,ki_x,kd_x
    kp_x = msg.data[0]
    ki_x =  msg.data[1]
    kd_x = msg.data[2]

def setPID_y(msg):
    global kp_y,ki_y,kd_y
    kp_y = msg.data[0]
    ki_y =  msg.data[1]
    kd_y = msg.data[2]

def setPID_z(msg):
    global kp_z,ki_z,kd_z
    kp_z = msg.data[0]
    ki_z =  msg.data[1]
    kd_z = msg.data[2]

# def setPID(msg):
#     global kp_x,kp_y,kp_z,ki_x,ki_y,ki_z,kd_x,kd_y,kd_z
#     kp_z = msg.doubles[0].value
#     ki_z = msg.doubles[1].value
#     kd_z = msg.doubles[2].value
#     kp_x = msg.doubles[3].value
#     ki_x = msg.doubles[4].value
#     kd_x = msg.doubles[5].value
#     kp_y = msg.doubles[6].value
#     ki_y = msg.doubles[7].value
#     kd_y = msg.doubles[8].value

def calPos(msg):
    global x,y,altitude

    # print("\n Odo frame:",msg.header.frame_id)
    x = round(msg.pose.pose.position.x,2)
    y = round(msg.pose.pose.position.y,2)
    altitude = round(msg.pose.pose.position.z,2)
    print(altitude)
# We need current velocity of the model so that we know when to stop and when to go
def calAng(msg):
    global vel_x,vel_y,vel_z
    vel_x = round(msg.angular_velocity.x,2)
    vel_y = round(msg.angular_velocity.y,2)
    vel_z = round(msg.angular_velocity.z,2)
# We also need its current orientation for RPY respectively
def calOrientation(msg):
    global roll, pitch, yaw
    #the data recieved from the sensor in in quaternion form
    orientation = [ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    #So, we need to convert that data from quaternion to euler using an in-built function
    roll, pitch, yaw = euler_from_quaternion(orientation)
    roll = round(roll,2)
    pitch = round(pitch,2)
    yaw = round(yaw,2)

def calAcc(msg):
    global acc_x,acc_y,acc_z
    acc_x = round(msg.linear_acceleration.x,2)
    acc_y = round(msg.linear_acceleration.y,2)
    acc_z = round(msg.linear_acceleration.z,2)
    print("\n Imu frame:",msg.header.frame_id)

def alt_control(odo, imu):
    # Set all variables to global so as to keep them updated values
    global altitude,req_alt,flag,roll, pitch, yaw,target_x,target_y, roll_desired, pitch_desired, yaw_desired,speed
    #So here we take readings from the IMU->Orientation and Odometry->(current_velocity & current_position) sensors
    calOrientation(odo)

    calPos(odo)

    calAng(imu)

    calAcc(imu)

    # setPID(pid)
    # message_filters.Subscriber("/dynamic_tutorials/parameter_updates", Config, setPID)

    # calAlt(gps)
    kap = 8.06428e-05 # 0.00099 #> constant for the matrix
    Mu = 7.2e-06  # 0.00004311  #> constant for the matrix

    rospy.Subscriber("pid_x", Float64MultiArray, setPID_x)
    rospy.Subscriber("pid_y", Float64MultiArray, setPID_y)
    rospy.Subscriber("pid_z", Float64MultiArray, setPID_z) 
    rospy.Subscriber("Tuning_Parameter", Float64, set_tuning_parameter)
    rospy.Subscriber("Rate_Controller_Gain", Float64, set_rate_controller_gain)
    #Making tuples for the velcities and target
    k_pose = (kp_x,ki_x,kd_x,kp_y,ki_y,kd_y,kp_z,ki_z,kd_z)
    # print(k_pose)
    target = (target_x,target_y,req_alt)
    velocity = (vel_x,vel_y,vel_z)
    acceleration = np.array([   [-acc_x],
                                [-acc_y],
                                [(acc_z-9.8)]   ])

    # print("acceleration:",acceleration)
    # print("velocity:",velocity)
    # Logging for debugging purposes
    # print("\nAltitude = " + str(altitude))
    # print("Required alt = ",req_alt)
    # print("Roll =", roll)
    # print("Pitch =", pitch)
    # print("Yaw =", yaw)
    # print("X = ",x)
    # print("Y = ",y)
    # sending the data to the PID_alt function which then calculates the speed using them
    speed = PID_alt(roll, pitch, yaw, x, y, target, altitude, flag, roll_desired, pitch_desired, yaw_desired, k_pose, velocity, kap, Mu, kq, kr, t1,speed,acceleration)
    flag += 1
    speed_pub.publish(speed)


#defining the control function to assign rotor speeds to the omav
def control():
    #We can get literally everything we need from the odometry sensor alone, but for extreme real life case we are taking atleast two sensors to start with, so that we are less proned to errors
    rospy.init_node('controller_node', anonymous=False)
    odo_sub = message_filters.Subscriber("/omav/odometry_sensor1/odometry", Odometry)
    imu_sub = message_filters.Subscriber("omav/imu",Imu)
    # pid_sub = message_filters.Subscriber("/dynamic_tutorials/parameter_updates", Config)
    tr = message_filters.TimeSynchronizer([odo_sub,imu_sub],2) #2 specifies the number of messages it should take from each sensor
    tr.registerCallback(alt_control)
    rospy.spin()

if __name__=='__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass