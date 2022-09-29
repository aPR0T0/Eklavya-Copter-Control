#! /usr/bin/env python3
import rospy
import numpy as np
import math
import message_filters
from std_msgs.msg import Float64MultiArray,Float64
from mav_msgs.msg import Actuators

def get_speed_publisher(F_dec, Mu, flag):
    """
    
    """
    #
    speed = Actuators()

    # 
    global N_Intermediate, N_Combined, Tilt

    #
    if(flag == 0):
        N_Intermediate = np.zeros(6)
        N_Combined = np.zeros(6)
        Tilt = np.zeros(6)


    # Combined Angular Velocity at a Rotor_Point
    i = 0
    for i in range(6):
        N_Intermediate[i] = math.sqrt((math.sqrt(pow(F_dec[i][0],2) + pow(F_dec[i+1][0],2))).real)

    #print("N_intermediate")
    #print(N_Intermediate)

    if(N_Intermediate[0] > 3000): N_Intermediate[0] = 3000
    if(N_Intermediate[0] < 40): N_Intermediate[0] = 40
    if(N_Intermediate[1] > 3000): N_Intermediate[1] = 3000
    if(N_Intermediate[1] < 40): N_Intermediate[1] = 40
    if(N_Intermediate[2] > 3000): N_Intermediate[2] = 3000
    if(N_Intermediate[2] < 40): N_Intermediate[2] = 40
    if(N_Intermediate[3] > 3000): N_Intermediate[3] = 3000
    if(N_Intermediate[3] < 40): N_Intermediate[3] = 40
    if(N_Intermediate[4] > 3000): N_Intermediate[4] = 3000
    if(N_Intermediate[4] < 40): N_Intermediate[4] = 40
    if(N_Intermediate[5] > 3000): N_Intermediate[5] = 3000
    if(N_Intermediate[5] < 40): N_Intermediate[5] = 40

    N_Combined = (0.5*N_Intermediate)

    # Angles of Tilt_Rotors
    i = 0
    for i in range(6):
        Tilt[i] = math.atan2((F_dec[i+1][0]).real, (F_dec[i][0]).real)
    # Giving Angular Velocity and Tilt Angle to Respective Rotor
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
    speed.angular_velocities[0] = (N_Combined[4])
    speed.angular_velocities[1] = (N_Combined[1])
    speed.angular_velocities[2] = (N_Combined[0])
    speed.angular_velocities[3] = (N_Combined[3])
    speed.angular_velocities[4] = (N_Combined[5])
    speed.angular_velocities[5] = (N_Combined[2])
    speed.angular_velocities[6] = (N_Combined[4])
    speed.angular_velocities[7] = (N_Combined[1])
    speed.angular_velocities[8] = (N_Combined[0])
    speed.angular_velocities[9] = (N_Combined[3])
    speed.angular_velocities[10] = (N_Combined[5])
    speed.angular_velocities[11] = (N_Combined[2])
    speed.angular_velocities[12] = (Tilt[4])
    speed.angular_velocities[13] = (Tilt[1])
    speed.angular_velocities[14] = (Tilt[0])
    speed.angular_velocities[15] = (Tilt[3])
    speed.angular_velocities[16] = (Tilt[5])
    speed.angular_velocities[17] = (Tilt[2])
    print(speed.angular_velocities)
    return(speed)   