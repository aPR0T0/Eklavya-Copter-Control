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
from math import sqrt
from math import atan2
import time
import numpy as np 
import rospy
from speed import *
from std_msgs.msg import Float64, Float64MultiArray
from mav_msgs.msg import Actuators


kp_x = 0.1

ki_x = 0.0001

kd_x = 0.75

kp_y = 0.15

ki_y = 0.0001

kd_y = 0.75

kp_z = 0.75

ki_z = 0.0005

kd_z = 5

g = 9.81 # gravitational acceleration

kap = 8.06428e-05 #0.00099 #> constant for the matrix

Mu = 7.2e-06 #3.4e-6 #> constant for the matrix

t1 = 0.866025404 #> sqrt(3)/2

len = 0.3 #> assuming that length is 0.3m 

xz = 0.707

helperr = np.zeros(200)
helperr_x = np.zeros(50)
helperr_y = np.zeros(50)

def PID_alt(roll, pitch, yaw, x, y, target, altitude, flag, roll_desired, pitch_desired, yaw_desired, k_pose, ang_velocities, kap, Mu, kq, kr, t1, speed,  acceleration, orientation, velocities):
    #global variables are declared to avoid their values resetting to 0
    global prev_alt_err,iMem_alt,dMem_alt,pMem_alt,prevTime, ddMem_alt, prevdMem_alt
    global sample_time,current_time
    global target_x,target_y,req_alt, current_velocity_pose_mat, vel_x, vel_y, vel_z
    global prop_pos_mat, diff_pose_mat, i_pose_mat, omega, helperr, prev_pos_mat, prev_velocity_pose_mat, acceleration_pose_mat

    
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


    omega = np.array([[ang_velocities[0]],[ang_velocities[1]],[ang_velocities[2]]])
    vel_x, vel_y, vel_z = velocities[0] , velocities[1] , velocities[2]
    target_x = round(target[0],2)
    target_y = round(target[1],2)
    req_alt = target[2] 

    # setting time for the differential terms and for later applications too
    sample_time = 0.005
    current_time = time.time()
    altitude = altitude - 0.17
    #Controller for x and y. Sets setpoint pitch and roll as output depending upon the corrections given by PID
    position_controller(target_x, target_y, x, y, flag, kp_x, ki_x, kd_x, kp_y, ki_y, kd_y)

    err_pitch =  (math.pi/180) * pitch_desired  - (pitch)  #Changed this from setpoint roll to roll desired
    err_roll  =  (math.pi/180) * roll_desired   - (roll)  #Changed this from setpoint pitch to pitch desired
    err_yaw   =  (math.pi/180) * yaw_desired    - (yaw)  #for our application we don't want the hexacopter to yaw like at all

    curr_alt_err = req_alt - altitude

    # Publishing error values to be plotted in rqt
    alt_err_pub = rospy.Publisher("/alt_err", Float64, queue_size=10)
    alt_err_pub.publish(curr_alt_err)
    roll_err_pub = rospy.Publisher("/roll_err", Float64, queue_size=10)
    roll_err_pub.publish(err_roll)
    pitch_err_pub = rospy.Publisher("/pitch_err", Float64, queue_size=10)
    pitch_err_pub.publish(err_pitch)
    yaw_err_pub = rospy.Publisher("/yaw_err", Float64, queue_size=10)
    yaw_err_pub.publish(err_yaw)

    # if(abs(curr_alt_err) < 4 and abs(vel_z) > 0.5 and abs(curr_alt_err)>0.3):
    #     dampner_z = (1/vel_z) * 0.2
    #     print("DampnerZ: ", dampner_z)
    #     curr_alt_err = (abs(curr_alt_err) * 1.3  - dampner_z)
    #     if (curr_alt_err < 0):
    #         curr_alt_err = -curr_alt_err

    mass_total = 4.27 #Kg this I got from the urdf file

    weight = mass_total*g

    hover_speed  = 678 #just taking an assumption later will correct it based on the experimanetal data

    # Flag for checking for the first time the function is called so that values can initilized
    # because if we don't check so then, we will encounter the error that we had no prevtime only
    if flag == 0:
        prevTime = 0   
        prev_alt_err = 0
        iMem_alt = 0
        dMem_alt = 0
        prevdMem_alt = 0
        ddMem_alt = 0
        prev_pos_mat = np.array([   [0],
                                    [0],
                                [-altitude]])
        prev_velocity_pose_mat = np.array([ [0],
                                            [0],
                                            [0]])

    #defining time for the differential terms
    dTime = current_time - prevTime
    # print(current_time,prevTime) >> Only uncomment for debugging
    #defining all the differential terms

    dErr_alt = curr_alt_err - prev_alt_err

    # print(dTime)
# ================== Starting calculations for the error terms =================== #

    current_pose_mat = np.round_(np.array([ [x ],
                                            [-y],
                                        [-altitude]]),decimals=2)
    i = 0
    for i in range(200):
        if i<199:
            helperr[i] = helperr[i+1]
    helperr[199] = curr_alt_err 
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
    #calculating current acceleration for f_desired calculations
        current_velocity_pose_mat = ((current_pose_mat - prev_pos_mat)/dTime)
        acceleration_pose_mat = np.round_((current_velocity_pose_mat - prev_velocity_pose_mat)/dTime, decimals=2)

    #Updating previous error terms

    prev_alt_err = curr_alt_err
    prevTime = current_time
    # print(prev_alt_err)
    # Final output correction terms after combining PID
    # output_alt = pMem_alt + iMem_alt + kd_thrust*dMem_alt

    # Now, For tuning purposes we will be limiting output altitude
    
    # output_alt = 1 if output_alt > 1 else output_alt

    prev_velocity_pose_mat = current_velocity_pose_mat

    # As the y and z axes are flipped of the body frame w.r.t ground frame hence we need to reverse signs of y and z terms

    prop_pos_mat = np.round_(np.array([ [ pMem_x ],
                                        [ pMem_y],
                                        [pMem_alt]]),decimals=2) #position error matrix
    # print(prop_pos_mat)
    diff_pose_mat = np.round_(np.array([[dMem_x],
                                        [dMem_y],
                                        [dMem_alt]]),decimals=2)
    # print(diff_pose_mat)
    i_pose_mat = np.round_(np.array([   [iMem_x],
                                        [iMem_y],
                                        [iMem_alt]]),decimals=2)
    # print(i_pose_mat)
    tilt_ang, ang_vel_rot = control_allocation( roll, pitch, yaw, hover_speed, mass_total, weight, flag, roll_desired, pitch_desired, yaw_desired, kq, kr, Mu, kap, acceleration, orientation)
    
    # prev_pos_mat = current_pose_mat
    
    speed = speed_assign( tilt_ang, ang_vel_rot,speed)
    
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

def control_allocation( roll, pitch, yaw, hover_speed, mass_total, weight, flag, roll_desired, pitch_desired, yaw_desired, kq, kr, Mu, kap,acceleration,orientation):
    # F_des --> Force desired and M_des --> Desired moment
    global current_time, prevTime, dTime, t1
    
    
    theta = pitch 
    phi = roll 
    gamma = yaw
    
    
    if (flag == 0):
        prevTime = 0

    dTime = current_time - prevTime
    sample_time = 0.005
#<--------------Intertia matrix for the Moment desired calc-------------------------->
    # angular velocities
    # 3x1
    
    I = np.array([  [0.086 ,-3.4208e-05,2.4695e-05 ],
                    [   0   ,  0.088 ,-3.8826e-06],
                    [   0   ,      0    ,  0.16  ]]) 
    # The above matrix is already defined in the urdf
    # print(omega)

#===============================Defining Matrices==================================>#
    relation_matrix = force_calc(phi, theta, gamma, Mu, kap, len, t1, mass_total, prop_pos_mat, diff_pose_mat, i_pose_mat, acceleration,flag, roll_desired, pitch_desired, yaw_desired, roll, pitch, yaw , omega[0][0], omega[1][0], omega[2][0], I,kq,kr, dTime, orientation)
    # print(F_des)
    # print(A_pseudo_inv)

    # print(Final_mat)
    # Now, here we consider xci = w^2*cos(αi) and xsi = w^2*sin(αi) 
    
    relation_matrix = np.round_(relation_matrix,decimals = 2)
    relation_matrix = relation_matrix.reshape((24,1))

    # Now, we are going to get the angles and the velocities for the rotors
    #Note: that we have not before just considered the real values from sins and cos it may cause some problem

    # Angular velocties deduction
    ang_vel= np.zeros(12)
    i = 0
    for i in range(12):
        ang_vel[i]= round(abs((1/sqrt(Mu))*(sqrt(sqrt(pow(relation_matrix[2*i],2) + pow(relation_matrix[2*i+1],2))).real)), 2) # ang_vel^2 = sqrt((Xci)^2+(Xsi)^2))

    # Tilt Angles deduction
    tilt_ang = np.zeros(6)
    i = 0
    for i in range(6):
        x1 = relation_matrix[2*i+1]
        x2 = relation_matrix[2*i]
        # print(x1) Uses this to get the real value from the matrix
        tilt_ang[i] = round(atan2(x1,x2),2) # atan2(sin/cos)

    ang_vel_rot = ang_vel
    return tilt_ang, ang_vel_rot

"""
    Note : CW -> Clockwise Rotation and CCW -> Anti Clockwise Rotation or Counter clockwise Rotation
            Here, We are considering CCW as +ve and CW as -ve
            Also, Here we are considering output error = current position - previous position
    So now we have got total errors for the x and y that we are off.
    So now what we do is to get at x and y is the following:

    1. To get to the x we need to pitch either CW or CCW
        * Now if the difference in current_error and previous_error in x is greater than a certain constant let's say 2 units, that means we have surpassed the point x
            So, Now we need to pitch in the opposite direction of the output error in x and also in the opposite direction of the ang_velocities in x
        
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

    err_x = target_x - x
    err_y = target_y - y

    # if ( err_x >  1 ): err_x =  1
    # if ( err_y < -1 ): err_y = -1

    dErr_x = err_x - prevErr_x
    dErr_y = err_y - prevErr_y

    sample_time = 0.005

    i = 0
    for i in range(50):
        if i<49:
            helperr_x[i] = helperr_x[i+1]
    helperr_x[49] = err_x
    i = 0
    for i in range(50):
        if i<49:
            helperr_y[i] = helperr_y[i+1]
    helperr_y[49] = err_y 
    if(dTime >= sample_time):
        
        # Proportional terms
        pMem_x = kp_x*err_x
        pMem_y = kp_y*err_y

        # Integral terms
        iMem_x = ki_x*np.sum(helperr_x)*dTime
        iMem_y = ki_y*np.sum(helperr_y)*dTime

        if(iMem_x>10): iMem_x = 10
        if(iMem_x<-10): iMem_x=-10
        if(iMem_y>10): iMem_y = 10
        if(iMem_y<-10): iMem_y=-10

        #Derivative terms

        dMem_x = kd_x*(dErr_x / dTime)
        dMem_y = kd_y*(dErr_y / dTime)
        # print(dErr_y)
    #updating previous terms
    prevErr_x = err_x
    prevErr_y = err_y



    x_err_pub = rospy.Publisher("/x_err", Float64, queue_size=10)
    x_err_pub.publish(err_x)
    y_err_pub = rospy.Publisher("/y_err", Float64, queue_size=10)
    y_err_pub.publish(err_y)

    #equation for correction
    if(abs(err_x) < 4 and abs(vel_x) > 0.35 and abs(err_x)>0.6):
        dampner = (1/vel_x) * 0.2
        print("Dampner: ", dampner)
        err_x = (err_x * 1.2  - dampner) #in the direction opposite to velocity
        # err_x = 10 if (err_x > 10) else err_x 
        # err_x = -10 if (err_x < -10) else err_x 


    if(abs(err_y) < 4 and abs(vel_y) > 0.35 and abs(err_y>0.6)):
        dampner_y = (1/vel_y) * 0.1
        # err_y = (vel_y * 2.35  - dampner_y) #in the direction opposite to velocity
        err_y = (err_y * 2.1  - dampner_y) #in the direction opposite to velocity
        # err_y = 10 if (err_y>10) else err_y
        # err_y = -10 if (err_y <-10) else err_y