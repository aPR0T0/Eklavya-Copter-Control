#! /usr/bin/env python3
from re import L
import rospy
import numpy as np
import math
import message_filters
from std_msgs.msg import Float64MultiArray,Float64
from mav_msgs.msg import Actuators

def get_speed_publisher(F_dec, Mu, flag, splitting_constant):

    speed = Actuators()

    global F_dec_square, N_Intermediate, N_angles, N_previous


    if(flag == 0):
        F_dec_square = np.zeros((12, 1))
        N_Intermediate = np.zeros(6)
        
        N_previous = np.zeros(6)

        N_angles = np.zeros(6)


    if(flag != 0):
        # Angular Velocity of Rotor Calculations
        F_dec_square = np.square(F_dec)

        for i in range(0, 6):
            N_Intermediate[i] = np.round_((math.sqrt(1/Mu))*(math.pow((F_dec_square[i*2][0] + F_dec_square[(i*2) + 1][0]), 0.25)), decimals=2)

        N_Intermediate = (splitting_constant * N_Intermediate)


        # for j in range(0, 6):
        #     if(N_Intermediate[j] - N_previous[j] > 10):
        #         N_Intermediate[j] = N_previous[j] + 10
        #     elif(N_Intermediate[j] - N_previous[j] < -10):
        #         N_Intermediate[j] = N_previous[j] - 10


        # for k in range(0, 6):
        #     if(N_Intermediate[k] > 750):
        #         N_Intermediate[k] = 750
        #     elif(N_Intermediate[k] < 640):
        #         N_Intermediate[k] = 640


        # Tilt-Angles Calculations
        for l in range(0, 6):
            N_angles[l] = np.round_((math.atan2((F_dec[l+1][0]), (F_dec[l][0]))), decimals=2)


        N_Intermediate = np.round_(N_Intermediate, decimals = 2)

        # Giving Angular Velocity and Tilt Angle to Respective Rotor
        for m in range(0, 6):
            speed.angular_velocities.append(N_Intermediate[m])
        for n in range(0, 6):
            speed.angular_velocities.append(N_Intermediate[n])
        for o in range(0, 6):
            speed.angular_velocities.append(N_angles[o])

        for p in range(0, 6):
            N_previous[p] = N_Intermediate[p]

    return(speed)