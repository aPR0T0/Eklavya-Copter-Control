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
    global F_dec_square, N_Intermediate, N_Combined

    #
    if(flag == 0):
        F_dec_square = np.zeros((12, 1))
        N_Intermediate = np.zeros(6)
        N_Combined = np.zeros(18)

    #
    F_dec_square = np.square(F_dec)

    # Combined Angular Velocity at a Rotor_Point
    N_Intermediate[0] = (math.sqrt((1/Mu) * (math.sqrt(F_dec_square[0, 0] + F_dec_square[1, 0]))))
    N_Intermediate[1] = (math.sqrt((1/Mu) * (math.sqrt(F_dec_square[2, 0] + F_dec_square[3, 0]))))
    N_Intermediate[2] = (math.sqrt((1/Mu) * (math.sqrt(F_dec_square[4, 0] + F_dec_square[5, 0]))))
    N_Intermediate[3] = (math.sqrt((1/Mu) * (math.sqrt(F_dec_square[6, 0] + F_dec_square[7, 0]))))
    N_Intermediate[4] = (math.sqrt((1/Mu) * (math.sqrt(F_dec_square[8, 0] + F_dec_square[9, 0]))))
    N_Intermediate[5] = (math.sqrt((1/Mu) * (math.sqrt(F_dec_square[10, 0] + F_dec_square[11, 0]))))

    if(N_Intermediate[0] > 3000): N_Intermediate[0] = 3000
    if(N_Intermediate[0] < 20): N_Intermediate[0] = 20
    if(N_Intermediate[1] > 3000): N_Intermediate[1] = 3000
    if(N_Intermediate[1] < 20): N_Intermediate[1] = 20
    if(N_Intermediate[2] > 3000): N_Intermediate[2] = 3000
    if(N_Intermediate[2] < 20): N_Intermediate[2] = 20
    if(N_Intermediate[3] > 3000): N_Intermediate[3] = 3000
    if(N_Intermediate[3] < 20): N_Intermediate[3] = 20
    if(N_Intermediate[4] > 3000): N_Intermediate[4] = 3000
    if(N_Intermediate[4] < 20): N_Intermediate[4] = 20
    if(N_Intermediate[5] > 3000): N_Intermediate[5] = 3000
    if(N_Intermediate[5] < 20): N_Intermediate[5] = 20

    N_Combined[0] = ((N_Intermediate[0])/2)
    N_Combined[1] = ((N_Intermediate[0])/2)
    N_Combined[2] = ((N_Intermediate[1])/2)
    N_Combined[3] = ((N_Intermediate[1])/2)
    N_Combined[4] = ((N_Intermediate[2])/2)
    N_Combined[5] = ((N_Intermediate[2])/2)
    N_Combined[6] = ((N_Intermediate[3])/2)
    N_Combined[7] = ((N_Intermediate[3])/2)
    N_Combined[8] = ((N_Intermediate[4])/2)
    N_Combined[9] = ((N_Intermediate[4])/2)
    N_Combined[10] = ((N_Intermediate[5])/2)
    N_Combined[11] = ((N_Intermediate[5])/2)


    # Angles of Tilt_Rotors
    N_Combined[12] = (math.atan2(F_dec[1, 0], F_dec[0, 0]))
    N_Combined[13] = (math.atan2(F_dec[3, 0], F_dec[2, 0]))
    N_Combined[14] = (math.atan2(F_dec[5, 0], F_dec[4, 0]))
    N_Combined[15] = (math.atan2(F_dec[7, 0], F_dec[6, 0]))
    N_Combined[16] = (math.atan2(F_dec[9, 0], F_dec[8, 0]))
    N_Combined[17] = (math.atan2(F_dec[11, 0], F_dec[10, 0]))


    # Giving Angular Velocity and Tilt Angle to Respective Rotor
    speed.angular_velocities.append(N_Combined[4])
    speed.angular_velocities.append(N_Combined[10])
    speed.angular_velocities.append(N_Combined[8])
    speed.angular_velocities.append(N_Combined[2])
    speed.angular_velocities.append(N_Combined[6])
    speed.angular_velocities.append(N_Combined[0])
    speed.angular_velocities.append(N_Combined[5])
    speed.angular_velocities.append(N_Combined[11])
    speed.angular_velocities.append(N_Combined[9])
    speed.angular_velocities.append(N_Combined[3])
    speed.angular_velocities.append(N_Combined[7])
    speed.angular_velocities.append(N_Combined[1])
    speed.angular_velocities.append(N_Combined[14])
    speed.angular_velocities.append(N_Combined[17])
    speed.angular_velocities.append(N_Combined[16])
    speed.angular_velocities.append(N_Combined[13])
    speed.angular_velocities.append(N_Combined[15])
    speed.angular_velocities.append(N_Combined[12])


    return(speed)