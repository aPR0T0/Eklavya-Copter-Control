#! /usr/bin/env python3

import rospy
import message_filters
from rospy.topics import Publisher
from std_msgs.msg import Float64, Float64MultiArray
from mav_msgs.msg import Actuators  
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# Initilization of all Parameters
altitude = 0
thrust = 0
#vel_x = 0 
#vel_y = 0 
#vel_z = 0
roll = 0
pitch = 0
yaw = 0
x = 0
y = 0
#z  = 0
time = 0
secs = 0
nsecs = 0

# Giving default PID values incase no input from user
kp = 20
ki = 0.001
kd = 35
kp_roll = 0.2
ki_roll = 0.00001
kd_roll = 0.5
kp_pitch = 0.15
ki_pitch = 0.00001
kd_pitch = 0.1
kp_yaw = 50
ki_yaw = 0.01
kd_yaw = 5
kp_x = 0.13
ki_x = 0.00001
kd_x =  0.003 #0.00015
kp_y = 0.13
ki_y = 0
kd_y = 0.00015
#kp_vel_x = 0.1
#ki_vel_x = 0
#kd_vel_x = 0.071
#kp_vel_y = 0.01
#ki_vel_y = 0.0
#kd_vel_y = 0.0071
# Flag for checking for the first time the script is run
flag = 0

# Message to publish final motor speeds to propellers
pub = rospy.Publisher('/firefly/command/motor_speed', Actuators, queue_size=1000)

# Ask the user for the required target coordinates the drone should hover at
target_x,target_y,req_alt = map(float,input("Enter X,Y,Z coordinates of target : ").split())

# Gets altitude PID published to node
def setPID_alt(msg):
    global kp,ki,kd
    kp = msg.data[0]
    ki =  msg.data[1]
    kd = msg.data[2]


# Gets roll PID published to node
def setPID_roll(msg):
    global kp_roll,ki_roll,kd_roll
    kp_roll = msg.data[0]
    ki_roll =  msg.data[1]
    kd_roll = msg.data[2]


# Gets pitch PID published to node
def setPID_pitch(msg):
    global kp_pitch,ki_pitch,kd_pitch
    kp_pitch = msg.data[0]
    ki_pitch =  msg.data[1]
    kd_pitch = msg.data[2]


# Gets yaw PID published to node
def setPID_yaw(msg):
    global kp_yaw,ki_yaw,kd_yaw
    kp_yaw = msg.data[0]
    ki_yaw =  msg.data[1]
    kd_yaw = msg.data[2]

# Gets x PID published to node
def setPID_x(msg):
    global kp_x,ki_x,kd_x
    kp_x = msg.data[0]
    ki_x = msg.data[1]
    kd_x = msg.data[2]

 # Gets y PID published to node   
def setPID_y(msg):
    global kp_y,ki_y,kd_y
    kp_y = msg.data[0]
    ki_y = msg.data[1]
    kd_y = msg.data[2]


def getOdo(msg):

    global time, secs, nsecs
    secs = msg.header.stamp.secs
    nsecs = (msg.header.stamp.nsecs)
    time = secs + (nsecs/1000000000)
    #rospy.loginfo("Time = {0}".format(time))

    global x, y, altitude
    x = round(msg.pose.pose.position.x, 3)
    y = round(msg.pose.pose.position.y, 3)
    altitude = round(msg.pose.pose.position.z, 3)
    #rospy.loginfo("X: {0}, Y: {1} & Altitude: {2}".format(x, y, altitude))

    orinetation_list = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
    global roll, pitch, yaw 
    (roll,pitch,yaw) = euler_from_quaternion(orinetation_list)
    roll = roll * (180/3.14159265)
    pitch = pitch * (180/3.14159265)
    yaw = yaw * (180/3.14159265)
    #rospy.loginfo("\nRoll = {0}\nPitch = {1}\nYaw = {2}\n".format(roll,pitch,yaw))

def calImu(msg):
    print("Call")


def PID_alt(roll, pitch, yaw, x, y, target, altitude, k_alt, k_roll, k_pitch, k_yaw, k_x, k_y, flag, time, secs, nsecs): 
    #global variables are declared to avoid their values resetting to 0
    global prev_alt_err,iMem_alt,dMem_alt,pMem_alt
    global prevTime,kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, setpoint_roll,setpoint_pitch, sample_time,current_time
    global kp_x,ki_x,kd_x,kp_y,ki_y,kd_y,target_x,target_y,req_alt
    global kp_thrust, ki_thrust, kd_thrust
    global prevErr_x,prevErr_y,pMem_x,pMem_y,iMem_x,iMem_y,dMem_x,dMem_y
    global kp_x,ki_x,kd_x
    global kp_y,ki_y,kd_y
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    print("Entry")
    #Assigning PID values here.
    kp_thrust = k_alt[0]
    ki_thrust = k_alt[1]
    kd_thrust = k_alt[2]
    kp_roll = k_roll[0]
    ki_roll = k_roll[1]
    kd_roll = k_roll[2]
    kp_pitch = k_pitch[0]
    ki_pitch = k_pitch[1]
    kd_pitch = k_pitch[2]
    kp_yaw = k_yaw[0]
    ki_yaw = k_yaw[1]
    kd_yaw = k_yaw[2]
    kp_x = k_x[0]
    ki_x = k_x[1]
    kd_x = k_x[2]
    kp_y = k_y[0]
    ki_y = k_y[1]
    kd_y = k_y[2]
    setpoint_roll = 0  #this should change according to the desired r,p,y
    setpoint_pitch = 0  #this should change according to the desired r,p,y
    setpoint_yaw = 0
    target_x = target[0]
    target_y = target[1]
    req_alt = target[2]
    sample_time = 0.005 #sampling time
    current_time = time

    #Controller for x,y,vel_x and vel_y tuning. Sets setpoint pitch and roll as output depending upon the corrections given by PID
    #position_controller(target_x, target_y, x, y, velocity, k_vel, flag)

    #Calculating errors
    err_pitch = pitch - setpoint_pitch
    err_roll = roll - setpoint_roll
    err_yaw = yaw - setpoint_yaw
    current_alt_err = req_alt - altitude
    err_x = x - target_x
    err_y = y - target_y

    # Publishing error values to be plotted in rqt
    alt_err_pub = rospy.Publisher("/alt_err", Float64, queue_size=10)
    alt_err_pub.publish(current_alt_err)
    roll_err_pub = rospy.Publisher("/roll_err", Float64, queue_size=10)
    roll_err_pub.publish(err_roll)
    pitch_err_pub = rospy.Publisher("/pitch_err", Float64, queue_size=10)
    pitch_err_pub.publish(err_pitch)
    yaw_err_pub = rospy.Publisher("/yaw_err", Float64, queue_size=10)
    yaw_err_pub.publish(err_yaw)


    #Speed found from testing at which drone hovers at a fixed height
    hover_speed = 543.32
  
    # Flag for checking for the first time the function is called so that values can initilized
    if flag == 0:
        prevTime = 0
        prevErr_roll = 0
        prevErr_pitch = 0
        prevErr_yaw = 0
        pMem_roll = 0
        pMem_pitch = 0
        pMem_yaw = 0
        iMem_roll = 0
        iMem_pitch = 0
        iMem_yaw = 0
        dMem_roll = 0
        dMem_pitch = 0
        dMem_yaw = 0    
        prev_alt_err = 0
        pMem_alt = 0
        iMem_alt = 0
        dMem_alt = 0
        time = 0
        secs = 0
        nsecs = 0
        prevTime = 0
        prevErr_x = 0
        prevErr_y = 0
        pMem_x = 0
        pMem_y = 0
        iMem_x = 0
        iMem_y = 0
        dMem_x = 0
        dMem_y = 0

    #Define all differential terms
    dTime = current_time - prevTime 
    #print("dTime = ",dTime)
    dErr_alt = current_alt_err - prev_alt_err 
    dErr_pitch = err_pitch - prevErr_pitch
    dErr_roll = err_roll - prevErr_roll
    dErr_yaw = err_yaw - prevErr_yaw
    dErr_x = err_x - prevErr_x
    dErr_y = err_y - prevErr_y
    
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    if (dTime >= sample_time):
        # Proportional terms
        pMem_alt = current_alt_err#this is for thrust
        pMem_roll = kp_roll * err_roll
        pMem_pitch = kp_pitch * err_pitch
        pMem_yaw = kp_yaw * err_yaw
        pMem_x = kp_x*err_x
        pMem_y = kp_y*err_y
        

        #Integral Terms(e(t))
        iMem_alt += current_alt_err * dTime #this is for thrust
        iMem_roll += err_roll* dTime
        iMem_pitch += err_pitch * dTime
        iMem_yaw += err_yaw * dTime

        iMem_x += err_x*dTime
        iMem_y += err_y*dTime

        #limit integrand values
        if(iMem_alt > 800): iMem_alt = 800
        if(iMem_alt <-800): iMem_alt = -800
        if(iMem_roll > 400): iMem_roll = 400
        if(iMem_roll < -400): iMem_roll = -400
        if(iMem_pitch > 10): iMem_pitch = 10
        if(iMem_pitch < -10): iMem_pitch = -10
        if(iMem_yaw > 400): iMem_yaw = 400
        if(iMem_yaw < -400): iMem_yaw = 400


        if(iMem_x>10): iMem_x = 10
        if(iMem_x<-10): iMem_x=-10
        if(iMem_y>10): iMem_y = 10
        if(iMem_y<-10): iMem_y=-10

        #Derivative Terms(e(t))
        dMem_roll = dErr_roll / dTime
        dMem_pitch = dErr_pitch / dTime
        dMem_yaw = dErr_yaw / dTime
        dMem_alt = dErr_alt / dTime
        
        dMem_x = dErr_x/dTime
        dMem_y = dErr_y/dTime



        prevTime = current_time

    # Updating all prev terms for next iteration
    prevErr_roll = err_roll
    prevErr_pitch = err_pitch
    prevErr_yaw = err_yaw
    prev_alt_err = current_alt_err
    prevErr_x = err_x
    prevErr_y = err_y

    # Final output correction terms after combining PID
    output_alt = kp_thrust*pMem_alt + ki_thrust*iMem_alt + kd_thrust*dMem_alt
    output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
    output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
    output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw 
    output_x = pMem_x + ki_x*iMem_x + kd_x*dMem_x
    output_y = pMem_y + ki_y*iMem_y + kd_y*dMem_y


    
        #setpoint_roll = output_vel_y
    # For Debugging Purposes
    # print("D Time = ",dTime)
    # print("Flag = ",flag)
    # print("P Term Alt= ",pMem_alt)
    # print("I Term Alt= ",iMem_alt)
    # print("D Term Alt= ",dMem_alt)
    # print("Altitude Error = ",current_alt_err)
    # print("Altitude Correction = ",output_alt)
    # print("P Term Roll= ",pMem_roll)
    # print("I Term Roll= ",iMem_roll)
    # print("D Term Roll= ",dMem_roll)
    # print("Roll Error = ",err_roll)
    # print("Roll Correction = ",output_roll)
    # print("P Term Pitch= ",pMem_pitch)
    # print("I Term Pitch= ",iMem_pitch)
    # print("D Term Pitch= ",dMem_pitch)
    # print("Pitch Error = ",err_pitch)
    # print("Pitch Correction = ",output_pitch)
    # print("P Term Yaw= ",pMem_yaw)
    # print("I Term Yaw= ",iMem_yaw)
    # print("D Term Yaw= ",dMem_yaw)
    # print("Yaw Error = ",err_yaw)
    # print("Yaw Correction = ",output_yaw)

    # Final thrust
    thrust = hover_speed + output_alt*2.5
    #Limiting thrust
    if(thrust > 800): 
        thrust = 800
    elif(thrust < 10):
        thrust = 10    
    # print("Thrust = ",thrust)

    #uncomment for only altitutde PID testing
    # output_roll=0 
    # output_pitch=0
    # output_yaw=0

    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    global speed_publisher
    speed_publisher = Actuators()

    speed_publisher.angular_velocities.append(thrust - output_yaw + output_pitch - output_roll)
    speed_publisher.angular_velocities.append(thrust + output_yaw + 0 - 0)
    speed_publisher.angular_velocities.append(thrust + output_yaw - output_pitch - output_roll)
    speed_publisher.angular_velocities.append(thrust - output_yaw - output_pitch + output_roll)
    speed_publisher.angular_velocities.append(thrust - output_yaw - 0 + 0)
    speed_publisher.angular_velocities.append(thrust + output_yaw + output_pitch + output_roll)

    # Limit Speed
    if(speed_publisher.angular_velocities[0] > 800): speed_publisher.angular_velocities[0] = 800
    if(speed_publisher.angular_velocities[1] > 800): speed_publisher.angular_velocities[1] = 800
    if(speed_publisher.angular_velocities[2] > 800): speed_publisher.angular_velocities[2] = 800
    if(speed_publisher.angular_velocities[3] > 800): speed_publisher.angular_velocities[3] = 800
    if(speed_publisher.angular_velocities[4] > 800): speed_publisher.angular_velocities[4] = 800
    if(speed_publisher.angular_velocities[5] > 800): speed_publisher.angular_velocities[5] = 800

    if(speed_publisher.angular_velocities[0] < 10): speed_publisher.angular_velocities[0] = 10
    if(speed_publisher.angular_velocities[1] < 10): speed_publisher.angular_velocities[1] = 10
    if(speed_publisher.angular_velocities[2] < 10): speed_publisher.angular_velocities[2] = 10
    if(speed_publisher.angular_velocities[3] < 10): speed_publisher.angular_velocities[3] = 10
    if(speed_publisher.angular_velocities[4] < 10): speed_publisher.angular_velocities[4] = 10
    if(speed_publisher.angular_velocities[5] < 10): speed_publisher.angular_velocities[5] = 10
    #rospy.loginfo(speed_publisher) print("Data")
    print(time)
    print(iMem_alt)
    print("Exit")
    print("Exit")
    #print(speed_publisher)
    return(speed_publisher)



    

def alt_control(imu):
    # Set all variables to global so as to keep them updated values
    global altitude,req_alt,flag, kp,ki,kd,roll, pitch, yaw,target_x,target_y, time, secs, nsecs

    #Gets drones current data
    rospy.Subscriber("/firefly/odometry_sensor1/odometry", Odometry, getOdo)

    calImu(imu)


    # Subsribe to all required topics to get PID for all controllers
    #rospy.Subscriber("alt_pid", Float64MultiArray, setPID_alt) 
    #rospy.Subscriber("roll_pid", Float64MultiArray, setPID_roll) 
    #rospy.Subscriber("pitch_pid", Float64MultiArray, setPID_pitch) 
    #rospy.Subscriber("yaw_pid", Float64MultiArray, setPID_yaw) 
    #rospy.Subscriber("x_pid", Float64MultiArray, setPID_x) 
    #rospy.Subscriber("y_pid", Float64MultiArray, setPID_y) 
    #rospy.Subscriber("vel_x_pid", Float64MultiArray, setPID_vel_x) 
    #rospy.Subscriber("vel_y_pid", Float64MultiArray, setPID_vel_y) 

    # Combine the PID values into tuples so as to easily send easily to PID function
    k_alt = (kp,ki,kd)
    k_roll = (kp_roll,ki_roll,kd_roll)
    k_pitch = (kp_pitch,ki_pitch,kd_pitch)
    k_yaw = (kp_yaw,ki_yaw,kd_yaw)
    k_x = (kp_x,ki_x,kd_x)
    k_y = (kp_y,ki_y,kd_y)
    #velocity = (vel_x, vel_y, vel_z)
    #k_vel = (kp_vel_x,ki_vel_x,kd_vel_x,kp_vel_y,ki_vel_y,kd_vel_y)
    target = (target_x,target_y,req_alt)

    # Logging for debugging purposes
    print("\nAltitude = " + str(altitude))
    print("Required alt = ",req_alt)
    #print("Roll =", roll)
    #print("Pitch =", pitch)
    #print("Yaw =", yaw)
    #print("X = ",x)
    #print("Y = ",y)
    
    #the goal is to get a function that stabilises the r p y x and y of the drone as per the given target while maintaining altitude
    #speed returned is the final motor speed after going through the motor mixing algorithm for all controllers
    global speed_publisher  
    speed_publisher = PID_alt(roll, pitch, yaw,x,y, target, altitude, k_alt, k_roll, k_pitch, k_yaw, k_x, k_y, flag, time, secs, nsecs)
    
    print(speed_publisher)
    
    flag += 1 #Indicates completion of 1st run of function

    # Publish the final motor speeds to the propellers
    pub.publish(speed_publisher)
    print("Publish")

def control():
    # define global values for required parameters to avoid resetting to 0
    global altitude, thrust, speed

    # initialize node
    rospy.init_node("altitude", anonymous = False)
    #rate = rospy.Rate(1)
    #rate.sleep()  
    imu_sub = message_filters.Subscriber("/firefly/imu", Imu)
    ts = message_filters.TimeSynchronizer([imu_sub], 10)
    ts.registerCallback(alt_control)

    rospy.spin()
       

# Main function
if __name__=='__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
        


