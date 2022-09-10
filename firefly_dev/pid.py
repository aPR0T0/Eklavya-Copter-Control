#! /usr/bin/env python3

import rospy
import message_filters
from rospy.topics import Publisher
from std_msgs.msg import Float64, Float64MultiArray
from mav_msgs.msg import Actuators  
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#Gets the target values and current values of rpy,x,y adn alt; and applies PID to all the errors and minimizes them
def PID_alt(roll, pitch, yaw, x, y, target, altitude, k_alt, k_roll, k_pitch, k_yaw, k_x, k_y, flag, time, secs, nsecs): 
    #global variables are declared to avoid their values resetting to 0
    global prev_alt_err,iMem_alt,dMem_alt,pMem_alt
    global prevTime,kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, setpoint_roll,setpoint_pitch, sample_time,current_time
    global kp_x,ki_x,kd_x,kp_y,ki_y,kd_y,target_x,target_y,req_alt
    global kp_thrust, ki_thrust, kd_thrust
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

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

    #Define all differential terms
    dTime = current_time - prevTime 
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

        #Derivative Terms(e(t))
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
    output_alt = kp_thrust*pMem_alt + ki_thrust*iMem_alt + kd_thrust*dMem_alt
    output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
    output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
    output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw 

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
    rospy.loginfo(speed_publisher) 

    return(speed_publisher)




