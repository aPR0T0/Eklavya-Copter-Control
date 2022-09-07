#! /usr/bin/env python3
# Giving default PID values incase no input from user

"""
        #> - means that the particular value needs to be changed while tuning

        d(X)  =  Derivative
        dd(X) =  Double Derivative
        _mat  =  Matrix
"""
import moment_force_allocation
from cmath import cos, sin, sqrt
from math import atan2
import time
import numpy as np 
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from mav_msgs.msg import Actuators
kp_thrust = 20
ki_thrust = 0.001
kd_thrust = 35
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
kp_vel_x = 0.1
ki_vel_x = 0
kd_vel_x = 0.071
kp_vel_y = 0.01
ki_vel_y = 0.0
kd_vel_y = 0.0071
g = 9.81 #gravitational acceleration
kap = 3 #> constant for the matrix
Mu = 3 #> constant for the matrix
t1 = 0.86603 #> sqrt(3)/2
len = 0.3 #> assuming that length is 0.3m 
xz = 0.55
def PID_alt(roll, pitch, yaw, x, y, target, altitude, velocity, flag, roll_desired, pitch_desired, yaw_desired):
    #global variables are declared to avoid their values resetting to 0
    global prev_alt_err,iMem_alt,dMem_alt,pMem_alt,prevTime, ddMem_alt, prevdMem_alt
    global kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, setpoint_roll,setpoint_pitch, sample_time,current_time
    global kp_x,ki_x,kd_x,kp_y,ki_y,kd_y,target_x,target_y,req_alt
    global kp_thrust, ki_thrust, kd_thrust, prop_pos_mat, diff_pose_mat, i_pose_mat, ddiff_pose_mat

    #Assigning target, altitude
    setpoint_roll = 0  #this should change according to the desired r,p,y
    setpoint_pitch = 0  #this should change according to the desired r,p,y
    
    #setting targets
    target_x = target[0]
    target_y = target[1]
    req_alt = target[2]
    k_vel = (kp_vel_x,ki_vel_x,kd_vel_x,kp_vel_y,ki_vel_y,kd_vel_y)
    # setting time for the differential terms and for later applications too
    sample_time = 0.005
    current_time = time.time()

    #Controller for x and y. Sets setpoint pitch and roll as output depending upon the corrections given by PID
    position_controller(target_x, target_y, x, y, velocity, k_vel, flag)

    err_pitch = pitch - setpoint_pitch 

    err_roll = roll - setpoint_roll

    err_yaw = 0 - yaw #for our application we don't want the hexacopter to yaw like at all

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

    #defining all the differential terms

    dErr_alt = curr_alt_err - prev_alt_err

    dErr_roll = err_roll - prevErr_roll

    dErr_pitch = err_pitch - prevErr_pitch

    dErr_yaw = err_yaw - prevErr_yaw


# ================== Starting calculations for the error terms =================== #


    if ( dTime >= sample_time ):

        # Proportional Terms
        
        pMem_alt = kp_thrust*curr_alt_err
        pMem_roll = kp_roll*err_roll
        pMem_pitch = kp_pitch*err_pitch
        pMem_yaw = kp_yaw*err_yaw
        
        # Integral Terms

        iMem_alt += ki_thrust*curr_alt_err*dTime
        iMem_roll += ki_roll*err_roll*dTime
        iMem_pitch += ki_pitch*err_pitch*dTime
        iMem_yaw += ki_yaw*err_yaw*dTime


        # Derivative Terms
        
        dMem_alt = dErr_alt / dTime
        dMem_roll = dErr_roll / dTime
        dMem_pitch = dErr_pitch / dTime
        dMem_yaw = dErr_yaw / dTime

        #limit integrand values
        if(iMem_alt > 800): iMem_alt = 800
        if(iMem_alt <-800): iMem_alt = -800
        if(iMem_roll > 400): iMem_roll = 400
        if(iMem_roll < -400): iMem_roll = -400
        if(iMem_pitch > 400): iMem_pitch = 400
        if(iMem_pitch < -400): iMem_pitch = -400
        if(iMem_yaw > 40): iMem_yaw = 40
        if(iMem_yaw < -40): iMem_yaw = 40


        ddMem_alt = (dMem_alt - prevdMem_alt) / dTime
    #Updating previous error terms

    prev_alt_err = curr_alt_err
    prevErr_roll = err_roll
    prevErr_pitch = err_pitch
    prevErr_yaw = err_yaw
    prevdMem_alt = dMem_alt

    # Final output correction terms after combining PID
    output_alt = pMem_alt + iMem_alt + kd_thrust*dMem_alt
    output_roll = pMem_roll +  iMem_roll + kd_roll * dMem_roll
    output_pitch = pMem_pitch + iMem_pitch + kd_pitch * dMem_pitch
    output_yaw = pMem_yaw + iMem_yaw + kd_yaw * dMem_yaw 

    # Now, For tuning purposes we will be limiting output altitude
    
    output_alt = 1 if output_alt > 2 else output_alt


    prop_pos_mat = np.matrix([[pMem_x],[pMem_y],[pMem_alt]]) #position error matrix
    
    diff_pose_mat = np.matrix([[dMem_x],[dMem_y],[dMem_alt]])

    i_pose_mat = np.matrix([[iMem_x],[iMem_y],[iMem_alt]])
    
    ddiff_pose_mat = np.matrix([[ddMem_x],[ddMem_y],[ddMem_alt]])

    control_allocation( roll, pitch, yaw, output_alt, output_roll, output_pitch, output_yaw, hover_speed, mass_total, weight, flag, roll_desired, pitch_desired, yaw_desired)


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

def control_allocation( roll, pitch, yaw, output_alt, output_roll, output_pitch, output_yaw, hover_speed, mass_total, weight, flag, roll_desired, pitch_desired, yaw_desired):
    global F_des, M_des, prevoutRoll, prevoutPitch, prevoutYaw # F_des --> Force desired and M_des --> Desired moment
    global dRoll, dPitch, dYaw, ang_vel_pitch, ang_vel_roll, ang_vel_yaw, ang_acc_pitch, ang_acc_roll, ang_acc_yaw
    global current_time,prevTime,dTime, Kp_pose, Ki_pose, Kd_pose, Final_mat, speed, prevOmega
    theta = output_pitch #required pitch
    phi = output_roll #required Roll
    gamma = output_yaw #required yaw
    prevOmega = np.zeros([3,1])
    Kp_pose = 0
    Ki_pose = 0
    Kd_pose = 2
    if (flag == 0):
        prevTime = 0
        prevoutRoll = 0
        prevoutPitch = 0
        prevoutYaw = 0
        prevOmega = np.zeros([3,1])

    dTime = current_time - prevTime
    sample_time = 0.005

    dRoll = phi - prevoutRoll
    dPitch = theta - prevoutPitch
    dYaw = gamma - prevoutYaw

    if (dTime >= sample_time):
        ang_vel_roll = dRoll / dTime
        ang_vel_pitch = dPitch / dTime
        ang_vel_yaw = dYaw / dTime
    
#===============================Defining Matrices==================================>#
    F_des, A_pseudo_inv = moment_force_allocation.force_desired(phi, theta, gamma, Mu, kap, len, t1, mass_total, prop_pos_mat, diff_pose_mat, i_pose_mat, ddiff_pose_mat)
    

#<--------------Intertia matrix for the Moment desired calc-------------------------->
    # angular velocities
    # 3x1
    omega = np.array([phi - gamma*sin(theta),ang_vel_pitch*cos(phi)+ang_vel_yaw*(cos(theta))*sin(phi),ang_vel_yaw*cos(phi)*cos(theta)-ang_vel_pitch*sin(phi)])
    
    I = np.array([[0.0075,0,0],[0,0.010939,0],[0,0,0.01369]]) 
    # The above matrix is already defined in the urdf
    
    M_des = moment_force_allocation.moment_desired(roll_desired, pitch_desired, yaw_desired, roll, pitch, yaw , omega[0], omega[1], omega[2], I)
    
    Final_mat = np.array([[F_des[0]],[F_des[1]],[F_des[2]],[M_des[0]],[M_des[1]],[M_des[2]]]) #6x1 matrix from Fdes and Mdes
    speed = Actuators()

    # Now, here we consider xci = w^2*cos(αi) and xsi = w^2*sin(αi) 
    relation_matrix = np.array(np.matmul( A_pseudo_inv , Final_mat ))
    
    # Now, we are going to get the angles and the velocities for the rotors
    #Note: that we have not before just considered the real values from sins and cos it may cause some problem
    

    # Angular velocties deduction
    ang_vel= np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    i = 0
    for i in range(6):
        ang_vel[i]= abs(sqrt(sqrt(pow(relation_matrix[2*i],2) + pow(relation_matrix[2*i+1],2))).real) # ang_vel^2 = sqrt((Xci)^2+(Xsi)^2))


    # Tilt Angles deduction
    tilt_ang = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    i = 0
    for i in range(6):
        x1 = pow(sqrt(relation_matrix[2*i+1]).real,2)
        x2 = pow(sqrt(relation_matrix[2*i]).real,2)
        # print(x1) Uses this to get the real value from the matrix
        tilt_ang[i] = atan2(x1,x2) # atan2(sin/cos)

    #Now, we need to allocate the speed to each rotor
    ang_vel_rot = xz*ang_vel
    t = 0
    # Uncomment for debugging only
    print(ang_vel_rot,tilt_ang)

    if ( t == 0 ):
        speed.angular_velocities.append(ang_vel_rot[4])
        speed.angular_velocities.append(ang_vel_rot[1])
        speed.angular_velocities.append(ang_vel_rot[0])
        speed.angular_velocities.append(ang_vel_rot[3])
        speed.angular_velocities.append(ang_vel_rot[5])
        speed.angular_velocities.append(ang_vel_rot[2])
        speed.angular_velocities.append(ang_vel_rot[4])
        speed.angular_velocities.append(ang_vel_rot[1])
        speed.angular_velocities.append(ang_vel_rot[0])
        speed.angular_velocities.append(ang_vel_rot[3])
        speed.angular_velocities.append(ang_vel_rot[5])
        speed.angular_velocities.append(ang_vel_rot[2])
        speed.angular_velocities.append(tilt_ang[4])
        speed.angular_velocities.append(tilt_ang[1])
        speed.angular_velocities.append(tilt_ang[0])
        speed.angular_velocities.append(tilt_ang[3])
        speed.angular_velocities.append(tilt_ang[5])
        speed.angular_velocities.append(tilt_ang[2])
        t += 1


    speed.angular_velocities[0] = ang_vel_rot[4]
    speed.angular_velocities[1] = ang_vel_rot[1]
    speed.angular_velocities[2] = ang_vel_rot[0]
    speed.angular_velocities[3] = ang_vel_rot[3]
    speed.angular_velocities[4] = ang_vel_rot[5]
    speed.angular_velocities[5] = ang_vel_rot[2]
    speed.angular_velocities[6] = ang_vel_rot[4]
    speed.angular_velocities[7] = ang_vel_rot[1]
    speed.angular_velocities[8] = ang_vel_rot[0]
    speed.angular_velocities[9] = ang_vel_rot[3]
    speed.angular_velocities[10] = ang_vel_rot[5]
    speed.angular_velocities[11] = ang_vel_rot[2]
    speed.angular_velocities[12] = tilt_ang[4]
    speed.angular_velocities[13] = tilt_ang[1]
    speed.angular_velocities[14] = tilt_ang[0]
    speed.angular_velocities[15] = tilt_ang[3]
    speed.angular_velocities[16] = tilt_ang[5]
    speed.angular_velocities[17] = tilt_ang[2]

    return(speed)
    


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
def position_controller(target_x, target_y, x, y, velocity, k_vel, flag):
    #global variables are declared to avoid their values resetting to 0
    global current_time,prevTime,dTime
    global prevErr_x,prevErr_y,pMem_x,pMem_y,iMem_x,iMem_y,dMem_x,dMem_y
    global kp_x,ki_x,kd_x,err_x,err_y,dErr_x,dErr_y
    global kp_y,ki_y,kd_y, prevdMem_x, prevdMem_y, ddMem_x, ddMem_y
    global setpoint_pitch, setpoint_roll
    
    vel_x = velocity[0]
    vel_y = velocity[1]
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
        ddMem_x = 0
        ddMem_y = 0

    #setting dTime for derivative and integral terms
    dTime = current_time - prevTime

    err_x = x - target_x
    err_y = y - target_y


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

        #Derivative terms

        dMem_x = kd_x*(dErr_x / dTime)
        dMem_y = kd_y*(dErr_y / dTime)


        ddMem_x = (dMem_x - prevdMem_x) / dTime
        ddMem_y = (dMem_y - prevdMem_y) / dTime
    #updating previous terms
    prevErr_x = err_x
    prevErr_y = err_y
    #the below given terms will help us find the rate of change of (rate of change of) position vector
    prevdMem_x = dMem_x
    prevdMem_y = dMem_y
    
    
    output_x = pMem_x + iMem_x + dMem_x
    output_y = pMem_y + iMem_y + dMem_y

    # Now we are setting the setpoint roll and pitches
    k = 2.0 #> 2.0
    if ( abs(err_x) > 2):
        setpoint_pitch = -(output_x)

    elif ( abs(err_x) < 2 and abs(vel_x) > 0.35 ): #> 0.35
        damp_vel = ( 1 / vel_x )*0.1 #> 0.1
        print( "damp_vel : ", damp_vel )
        setpoint_pitch = -(vel_x*k - damp_vel) #as specified earlier, setpoint should be opposite direction of the velocities
    
    setpoint_pitch = 10 if setpoint_pitch > 10 else setpoint_pitch
    setpoint_pitch = -10 if setpoint_pitch <-10 else setpoint_pitch

    if ( abs(err_y) > 2):
        setpoint_roll = -(output_y)

    elif ( abs(err_y) < 2 and abs(vel_y) > 0.35 ): #> 0.35
        damp_vel = ( 1 / vel_y )*0.1 #> 0.1
        print( "damp_vel : ", damp_vel )
        setpoint_roll = -(vel_y*k - damp_vel) #as specified earlier, setpoint should be opposite direction of the velocities
    
    setpoint_roll = 10 if setpoint_roll > 10 else setpoint_roll
    setpoint_roll = -10 if setpoint_roll <-10 else setpoint_roll