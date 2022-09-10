#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from mav_msgs.msg import Actuators  
from nav_msgs.msg import Odometry
#from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from rosgraph_msgs.msg import Clock

flag = 0
time = 0


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
kd_x =  0.003
kp_y = 0.13
ki_y = 0
kd_y = 0.00015
flag = 0



def getOdo(msg):

    global time, secs, nsecs
    secs = msg.header.stamp.secs
    nsecs = (msg.header.stamp.nsecs)
    time = secs + (nsecs/1000000000)
    #print("Time")
    #print(time)
    rospy.loginfo("Time = {0}".format(time))

    global x, y, altitude
    x = round(msg.pose.pose.position.x, 3)
    y = round(msg.pose.pose.position.y, 3)
    altitude = round(msg.pose.pose.position.z, 3)
    #rospy.loginfo("X: {0}, Y: {1} & Altitude: {2}".format(x, y, altitude))
    #print("X, Y & Z")
    #print(x, y, z)

    orinetation_list = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
    global roll, pitch, yaw 
    (roll,pitch,yaw) = euler_from_quaternion(orinetation_list)
    roll = roll * (180/3.14159265)
    pitch = pitch * (180/3.14159265)
    yaw = yaw * (180/3.14159265)

    
    #print("Roll, Pitch & Yaw")
    #print(roll, pitch, yaw)
    #rospy.loginfo("\nRoll = {0}\nPitch = {1}\nYaw = {2}\n".format(roll,pitch,yaw))

## Check this
#def listener():
    #rospy.init_node("fresh_control", anonymous = False)
    #rospy.Subscriber("/clock", Clock, getClock)
    #rospy.Subscriber("/firefly/imu", Imu, getImu)
    
    
        
    



pub = rospy.Publisher('/firefly/command/motor_speed', Actuators, queue_size=10)

def main_run(flag):

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
        flag = 0
        secs = 0
        nsecs = 0
            # initial values
        altitude = 0
        thrust = 0
        roll = 0
        pitch = 0
        yaw = 0
        x = 0
        y = 0
        z  = 0
        target_x = 0
        target_y = 0
        req_alt = 0

    target_x,target_y,req_alt = map(float,input("Enter X,Y,Altitude coordinates of target : ").split())

    setpoint_roll = 0
    setpoint_pitch = 0
    setpoint_yaw = 0

    sample_time = 0.005

    rospy.Subscriber("/firefly/odometry_sensor1/odometry", Odometry, getOdo)

    

    

    err_roll = roll - setpoint_roll
    err_pitch = pitch - setpoint_pitch
    err_yaw = yaw - setpoint_yaw
    current_alt_err = req_alt - altitude
    err_x = x - target_x
    err_y = y - target_y

    hover_speed = 543.32
        
    current_time = time

        #Define all differential terms
    dTime = current_time - prevTime 
    #rospy.loginfo(dTime)
    #print("dTime = ",dTime)
    dErr_alt = current_alt_err - prev_alt_err 
    dErr_pitch = err_pitch - prevErr_pitch
    dErr_roll = err_roll - prevErr_roll
    dErr_yaw = err_yaw - prevErr_yaw

        #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    if (dTime >= sample_time):
            # Proportional terms
        pMem_alt = current_alt_err#this is for thrust
        pMem_roll = kp_roll * err_roll
        pMem_pitch = kp_pitch * err_pitch
        pMem_yaw = kp_yaw * err_yaw

            #Integral Terms(e(t))
        iMem_alt += current_alt_err * dTime #this is for thrust
        iMem_roll += err_roll* dTime
        iMem_pitch += err_pitch * dTime
        iMem_yaw += err_yaw * dTime
            #limit integrand values
        if(iMem_alt > 800): iMem_alt = 800
        if(iMem_alt <-800): iMem_alt = -800
        if(iMem_roll > 400): iMem_roll = 400
        if(iMem_roll < -400): iMem_roll = -400
        if(iMem_pitch > 10): iMem_pitch = 10
        if(iMem_pitch < -10): iMem_pitch = -10
        if(iMem_yaw > 400): iMem_yaw = 400
        if(iMem_yaw < -400): iMem_yaw = 400

    if(dTime != 0):        #Derivative Terms(e(t))
        dMem_roll = dErr_roll / dTime
        dMem_pitch = dErr_pitch / dTime
        dMem_yaw = dErr_yaw / dTime
        dMem_alt = dErr_alt / dTime
        
    prevTime = current_time

    # Updating all prev terms for next iteration
    prevErr_roll = err_roll
    prevErr_pitch = err_pitch
    prevErr_yaw = err_yaw
    prev_alt_err = current_alt_err

    # Final output correction terms after combining PID
    output_alt = kp*pMem_alt + ki*iMem_alt + kd*dMem_alt
    output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
    output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
    output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw 

    thrust = hover_speed + output_alt*2.5
    #Limiting thrust
    if(thrust > 800): 
        thrust = 800
    elif(thrust < 10):
        thrust = 10    

    global speed_publisher
    speed_publisher = Actuators()
    print("Secs")
    print(secs)
    speed_publisher.header.stamp.secs = (secs)
    speed_publisher.header.stamp.nsecs = nsecs

    speed_publisher.angular_velocities.append(thrust - output_yaw + output_pitch - output_roll)
    speed_publisher.angular_velocities.append(thrust + output_yaw + 0 - 0)
    speed_publisher.angular_velocities.append(thrust + output_yaw - output_pitch - output_roll)
    speed_publisher.angular_velocities.append(thrust - output_yaw - output_pitch + output_roll)
    speed_publisher.angular_velocities.append(thrust - output_yaw - 0 + 0)
    speed_publisher.angular_velocities.append(thrust + output_yaw + output_pitch + output_roll)

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

    print("Data")
    print(time)
    print(iMem_alt)
    print("Exit")
    
    flag+=1

    rospy.init_node("fresh_control", anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo(speed_publisher) 
        pub.publish(speed_publisher)
        rate.sleep()


    


def publisher():
    flag = 0
    while(1):
        main_run(flag)
        flag+=1
        
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass




