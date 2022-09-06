#! /usr/bin/env python3

from cmath import pi
import rospy #for basically any type of ros operations
import time #to get definite time for the dTime
import message_filters
from std_msgs.msg import Float64MultiArray,Float64
from sensor_msgs.msg import Imu #It contains overall type of readings getting from the Imu sensor
from nav_msgs.msg import Odometry #It contains overall type of readings getting from the Odometry sensor
from mav_msgs.msg import Actuators
from tf.transformations import euler_from_quaternion, quaternion_from_euler
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

# Flag for checking for the first time the script is run
flag = 0

#creating publisher for the speeds of the rotors
speed_pub = rospy.Publisher("/omav/command/motor_speed",queue_size=100) #here we will use angles section to give angles to the tilt rotors

# Asking user for the desired coordinates
target_x, target_y, req_alt = map( float , input("Enter X Y (position) and Altitude : ".split()))
roll_desired, pitch_desired, yaw_desired = map( float , input("Enter desired orientation (in degrees) Roll, Pitch, Yaw (resp): ".split()))
# We need x,y and altitude of the model
# split() : Return a list of the words in the string, using sep as the delimiter string
# map() : Basically provides the value recieved in it to the variables on the left with the given data type()
def calPos(msg):
    global x,y,altitude
    x = round(msg.pose.pose.position.x,3)
    y = round(msg.pose.pose.position.y,3)
    altitude = round(msg.pose.pose.position.z,3)
# We need current velocity of the model so that we know when to stop and when to go
def calVel(msg):
    global vel_x,vel_y,vel_z
    vel_x = msg.twist.twist.linear.x
    vel_y = msg.twist.twist.linear.y
    vel_z = msg.twist.twist.linear.z
# We also need its current orientation for RPY respectively
def calOrientation(msg):
    global roll, pitch, yaw
    #the data recieved from the sensor in in quaternion form
    orientation = [ msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.ws]
    #So, we need to convert that data from quaternion to euler using an in-built function
    roll, pitch, yaw = euler_from_quaternion(orientation)
    #we need RPY in degrees
    roll = (roll*180)/pi
    pitch = (pitch*180)/pi
    yaw = (yaw*180)/pi

def alt_control(imu,odo):
    # Set all variables to global so as to keep them updated values
    global altitude,req_alt,flag,roll, pitch, yaw,target_x,target_y, roll_desired, pitch_desired, yaw_desired
    #So here we take readings from the IMU->Orientation and Odometry->(current_velocity & current_position) sensors
    calOrientation(imu)

    calPos(odo)

    calVel(odo)
    #Making tuples for the velcities and target
    velocity = (vel_x, vel_y, vel_z)
    target = (target_x,target_y,req_alt)


    # Logging for debugging purposes
    print("\nAltitude = " + str(altitude))
    print("Required alt = ",req_alt)
    print("Roll =", roll)
    print("Pitch =", pitch)
    print("Yaw =", yaw)
    print("X = ",x)
    print("Y = ",y)

    
    # sending the data to the PID_alt function which then calculates the speed using them
    speed = PID_alt(roll, pitch, yaw, x, y, target, altitude, velocity, flag, roll_desired, pitch_desired, yaw_desired)
    flag += 1

    speed_pub.publish(speed)


#defining the control function to assign rotor speeds to the omav
def control():
    #We can get literally everything we need from the odometry sensor alone, but for extreme real life case we are taking atleast two sensors to start with, so that we are less proned to errors
    rospy.init_node('controller_node', anonymous=False)
    #So here we take readings from the IMU->Orientation and Odometry->(current_velocity & current_position) sensors
    imu_sub = message_filters.Subscriber("/omav/ground_truth/imu", Imu)
    odo_sub = message_filters.Subscriber("/omav/ground_truth/Odometry", Odometry)
    tr = message_filters.TimeSynchronizer([imu_sub,odo_sub],2) #2 specifies the number of messages it should take from each sensor
    tr.registerCallback(alt_control)
    rospy.spin()

if __name__=='__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
