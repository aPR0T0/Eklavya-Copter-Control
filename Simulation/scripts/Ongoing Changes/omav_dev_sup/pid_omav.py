#! /usr/bin/env python3
# Giving default PID values incase no input from user

"""
        #> - means that the particular value needs to be changed while tuning

        d(X)  =  Derivative
        dd(X) =  Double Derivative
        _mat  =  Matrix
"""
from moment_desired import *
from moment_force_allocation import *
from cmath import cos, sin, sqrt
from math import atan2
import time
import numpy as np 
import rospy
from speed import *
from std_msgs.msg import Float64, Float64MultiArray
from mav_msgs.msg import Actuators


kp_x = 20

ki_x = 0

kd_x = 10

kp_y = 20

ki_y = 0

kd_y = 10

kp_z = 20

ki_z = 0

kd_z = 10

g = 9.81 # gravitational acceleration

kap = 8.06428e-05 #0.00099 #> constant for the matrix

Mu = 7.2e-06 #3.4e-6 #> constant for the matrix

t1 = 0.866025404 #> sqrt(3)/2

len = 0.3 #> assuming that length is 0.3m 

xz = 0.707


def PID_alt(roll, pitch, yaw, x, y, target, altitude, flag, roll_desired, pitch_desired, yaw_desired, k_pose, velocity, kap, Mu, kq, kr, t1,speed):
    #global variables are declared to avoid their values resetting to 0
    global prev_alt_err,iMem_alt,dMem_alt,pMem_alt,prevTime, ddMem_alt, prevdMem_alt
    global prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, setpoint_roll,setpoint_pitch, sample_time,current_time
    global target_x,target_y,req_alt
    global prop_pos_mat, diff_pose_mat, i_pose_mat, omega, helperr

    #Assigning target, altitude
    setpoint_roll = 0  #this should change according to the desired r,p,y
    setpoint_pitch = 0  #this should change according to the desired r,p,y
    
    #setting targets
    kp_x = k_pose[0]
    ki_x = k_pose[1]
    kd_x = k_pose[2]

    kp_y = k_pose[3]
    ki_y = k_pose[4]
    kd_y = k_pose[5]

    kp_z = k_pose[6]
    ki_z = k_pose[7]
    kd_z = k_pose[8]


    omega = np.array([[velocity[0]],[velocity[1]],[velocity[2]]])
    target_x = round(target[0],2)
    target_y = round(target[1],2)
    req_alt = target[2]

    # setting time for the differential terms and for later applications too
    sample_time = 0.005
    current_time = time.time()
    altitude = altitude - 0.17
    #Controller for x and y. Sets setpoint pitch and roll as output depending upon the corrections given by PID
    position_controller(target_x, target_y, x, y, flag, kp_x, ki_x, kd_x, kp_y, ki_y, kd_y)

    err_pitch = pitch - pitch_desired #Changed this from setpoint roll to roll desired

    err_roll = roll - roll_desired #Changed this from setpoint pitch to pitch desired

    err_yaw = 0 - yaw #for our application we don't want the hexacopter to yaw like at all

    curr_alt_err = req_alt - altitude
    # if(0.6 <= curr_alt_err < 2):
    #     curr_alt_err = curr_alt_err - (1/curr_alt_err)*0.4
    # elif( -2 < curr_alt_err <= -0.6):
    #     curr_alt_err = curr_alt_err - (1/curr_alt_err)*0.4
    #this is limiting case where we have reached the desired location in x and y
    # if(-2.5 <=setpoint_pitch <=2.5 and -2.5<= setpoint_roll <= 2.5): 
    #     err_roll = setpoint_roll
    #     err_pitch = setpoint_pitch

    # Publishing error values to be plotted in rqt
    alt_err_pub = rospy.Publisher("/alt_err", Float64, queue_size=10)
    alt_err_pub.publish(curr_alt_err)
    roll_err_pub = rospy.Publisher("/roll_err", Float64, queue_size=10)
    roll_err_pub.publish(err_roll)
    pitch_err_pub = rospy.Publisher("/pitch_err", Float64, queue_size=10)
    pitch_err_pub.publish(err_pitch)
    yaw_err_pub = rospy.Publisher("/yaw_err", Float64, queue_size=10)
    yaw_err_pub.publish(err_yaw)
    x_err_pub = rospy.Publisher("/x_err", Float64, queue_size=10)
    x_err_pub.publish(-err_x)
    y_err_pub = rospy.Publisher("/y_err", Float64, queue_size=10)
    y_err_pub.publish(err_y)


    mass_total = 4.04 #Kg this I got from the urdf file

    weight = 4.04*g

    hover_speed  = 678 #just taking an assumption later will correct it based on the experimanetal data

    # Flag for checking for the first time the function is called so that values can initilized
    # because if we don't check so then, we will encounter the error that we had no prevtime only
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
        iMem_alt = 0
        dMem_alt = 0
        prevdMem_alt = 0
        ddMem_alt = 0

    #defining time for the differential terms
    dTime = current_time - prevTime
    # print(current_time,prevTime) Only uncomment for debugging
    #defining all the differential terms

    dErr_alt = curr_alt_err - prev_alt_err

    # print(dTime)
# ================== Starting calculations for the error terms =================== #

    helperr = np.zeros(20)
    i = 0
    for i in range(20):
        if i<19:
            helperr[i] = helperr[i+1]
        else:
            helperr[i] = curr_alt_err 
    if ( dTime >= sample_time ):

        # Proportional Terms
        
        pMem_alt = kp_z*curr_alt_err
        
        # Integral Terms

        iMem_alt = ki_z*np.sum(helperr)*dTime


        # Derivative Terms
        
        dMem_alt = kd_z*(dErr_alt / dTime)

        #limit integrand values
        # print(iMem_alt)
        if(iMem_alt > 100): iMem_alt = 100
        if(iMem_alt < -100): iMem_alt = -100

    #Updating previous error terms

    prev_alt_err = curr_alt_err
    prevTime = current_time
    # print(prev_alt_err)
    # Final output correction terms after combining PID
    # output_alt = pMem_alt + iMem_alt + kd_thrust*dMem_alt

    # Now, For tuning purposes we will be limiting output altitude
    
    # output_alt = 1 if output_alt > 1 else output_alt


    prop_pos_mat = np.round_(np.array([[pMem_x],[pMem_y],[pMem_alt]]),decimals=2) #position error matrix
    print(prop_pos_mat)
    diff_pose_mat = np.round_(np.array([[dMem_x],[dMem_y],[dMem_alt]]),decimals=2)
    print(diff_pose_mat)
    i_pose_mat = np.round_(np.array([[iMem_x],[iMem_y],[iMem_alt]]),decimals=2)
    # print(i_pose_mat)
    tilt_ang, ang_vel_rot = control_allocation( roll, pitch, yaw, hover_speed, mass_total, weight, flag, roll_desired, pitch_desired, yaw_desired, kq, kr, Mu, kap)

    speed = speed_assign( tilt_ang, ang_vel_rot,speed,flag)
    
    return speed
# ======================= Control Allocation Starts here ========================== #

"""
<-----------------------------------Matrices Used------------------------------------->

    1. Rotation matrix: 
            R(from ground to the body frame) = R(phi)^T*R(theta)^T*R(gamma)^T
    2. Static Allocation matrix:
            A = [constants](6x12)
    3. Hybrid matrix:
            x = [ xc1   xs1   xc2   xs2   xc3   xs3   xc4   xs4   xc5   xs5   xc6   xs6]^T
            Where, xci = cos(αi) and xsi = sin(αi) (Here, αi = Tilt angles of the ith rotor)   
    4. We also need a pseudo inverse for the static allocation matrix

"""

def control_allocation( roll, pitch, yaw, hover_speed, mass_total, weight, flag, roll_desired, pitch_desired, yaw_desired, kq, kr, Mu, kap):
    # F_des --> Force desired and M_des --> Desired moment
    global current_time, prevTime, dTime, t1
    theta = pitch 
    phi = roll 
    gamma = yaw
    if (flag == 0):
        prevTime = 0
        prevoutRoll = 0
        prevoutPitch = 0
        prevoutYaw = 0
        prevOmega = np.zeros([3,1])

    dTime = current_time - prevTime
    sample_time = 0.005
#<--------------Intertia matrix for the Moment desired calc-------------------------->
    # angular velocities
    # 3x1
    
    I = np.array([[0.0075,-3.4208e-05,2.4695e-05],[0,0.010939,-3.8826e-06],[0,0,0.01369]]) 
    # The above matrix is already defined in the urdf

#===============================Defining Matrices==================================>#
    relation_matrix = force_calc(phi, theta, gamma, Mu, kap, len, t1, mass_total, prop_pos_mat, diff_pose_mat, i_pose_mat, flag, roll_desired, pitch_desired, yaw_desired, roll, pitch, yaw , omega[0][0], omega[1][0], omega[2][0], I,kq,kr)
    # print(F_des)
    # print(A_pseudo_inv)

    # print(Final_mat)
    # Now, here we consider xci = w^2*cos(αi) and xsi = w^2*sin(αi) 
    
    relation_matrix = np.round_(relation_matrix,decimals = 2)
    relation_matrix = relation_matrix.reshape((12,1))

    # Now, we are going to get the angles and the velocities for the rotors
    #Note: that we have not before just considered the real values from sins and cos it may cause some problem

    # Angular velocties deduction
    ang_vel= np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    i = 0
    for i in range(6):
        ang_vel[i]= round(abs((1/sqrt(Mu))*(sqrt(sqrt(pow(relation_matrix[2*i],2) + pow(relation_matrix[2*i+1],2))).real)), 2) # ang_vel^2 = sqrt((Xci)^2+(Xsi)^2))


    # Tilt Angles deduction
    tilt_ang = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    i = 0
    for i in range(6):
        x1 = relation_matrix[2*i+1]
        x2 = relation_matrix[2*i]
        # print(x1) Uses this to get the real value from the matrix
        tilt_ang[i] = round(atan2(x1,x2),2) # atan2(sin/cos)

    #Now, we need to allocate the speed to each rotor
    ang_vel_rot = tuple(xz*ang_vel)
    # Uncomment for debugging only
    # print(ang_vel_rot,tilt_ang)
    tilt_ang = tuple(tilt_ang)
    return tilt_ang, ang_vel_rot

"""
    Note : CW -> Clockwise Rotation and CCW -> Anti Clockwise Rotation or Counter clockwise Rotation
            Here, We are considering CCW as +ve and CW as -ve
            Also, Here we are considering output error = current position - previous position
    So now we have got total errors for the x and y that we are off.
    So now what we do is to get at x and y is the following:

    1. To get to the x we need to pitch either CW or CCW
        * Now if the difference in current_error and previous_error in x is greater than a certain constant let's say 2 units, that means we have surpassed the point x
            So, Now we need to pitch in the opposite direction of the output error in x and also in the opposite direction of the velocity in x
        
    2. To get to the y we need to roll either CW or CCW
        * Now if the difference in current_error and previous_error in y is greater than a certain constant let's say 2 units, that means we have surpassed the point y
            So, Now we need to roll in the opposite direction of the output error in y and also in the opposite direction of the velociy in y

"""

#Controller which applies PID to errors in x and y(target values of vel being 0) and gives setpoint pitch and roll as output to correct the errors
def position_controller(target_x, target_y, x, y, flag, kp_x, ki_x, kd_x, kp_y, ki_y, kd_y):
    #global variables are declared to avoid their values resetting to 0
    global prevTime,dTime
    global prevErr_x,prevErr_y,pMem_x,pMem_y,iMem_x,iMem_y,dMem_x,dMem_y
    global err_x,err_y,dErr_x,dErr_y
    global prevdMem_x, prevdMem_y
    global setpoint_pitch, setpoint_roll
    
    if (flag == 0):
        prevTime = 0
        prevErr_x = 0
        prevErr_y = 0
        prevdMem_x = 0
        prevdMem_y = 0
        pMem_x = 0
        pMem_y = 0
        iMem_x = 0
        iMem_y = 0
        dMem_x = 0
        dMem_y = 0

    #setting dTime for derivative and integral terms
    dTime = current_time - float(prevTime)

    err_x = x - target_x
    err_y = target_y - y

    dErr_x = err_x - prevErr_x
    dErr_y = err_y - prevErr_y

    sample_time = 0.005

    if(dTime >= sample_time):
        
        # Proportional terms
        pMem_x = kp_x*err_x
        pMem_y = kp_y*err_y

        # Integral terms
        iMem_x += ki_x*err_x*dTime
        iMem_y += ki_y*err_y*dTime

        if(iMem_x>10): iMem_x = 10
        if(iMem_x<-10): iMem_x=-10
        if(iMem_y>10): iMem_y = 10
        if(iMem_y<-10): iMem_y=-10

        #Derivative terms

        dMem_x = kd_x*(dErr_x / dTime)
        dMem_y = kd_y*(dErr_y / dTime)
        print(dErr_y)
    #updating previous terms
    prevErr_x = err_x
    prevErr_y = err_y

    # damping 

    # if (0 < err_x < 2):
    #     damper = 3*abs(err_x)/4
    #     print("\ndamping in x:",damper)
    #     err_x = err_x + damper #because the direction of x is same as that of earth's frame
    # elif (-2 < err_x < 0):
    #     damper = 3*abs(err_x)/4
    #     print("\ndamping in x:",damper)
    #     err_x = err_x - damper
    # print("\nerr_x = ",err_x)


    # if (0 < err_y < 2):
    #     damper = 3*(1/abs(err_y))/4
    #     print("\ndamping in y:",damper)
    #     err_y = err_y - damper
    # elif (-2 < err_y < 0):
    #     damper = 3*(1/abs(err_y))/4
    #     print("\ndamping in y:",damper)
    #     err_y = err_y + damper
    # print("\nerr_y = ",err_y)