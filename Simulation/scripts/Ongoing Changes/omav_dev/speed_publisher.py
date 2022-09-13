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
    global F_dec_square, N_Intermediate, N_Combined, t_dash

    #
    if(flag == 0):
        F_dec_square = np.zeros((12, 1))
        N_Intermediate = np.zeros(6)
        N_Combined = np.zeros(6)
        Tilt = np.zeros(6)

    #
    F_dec_square = np.square(F_dec)

    # Combined Angular Velocity at a Rotor_Point
    N_Intermediate[0] = (math.sqrt((1/Mu) * math.sqrt((math.sqrt(F_dec_square[0, 0] + F_dec_square[1, 0])))))
    N_Intermediate[1] = (math.sqrt((1/Mu) * math.sqrt((math.sqrt(F_dec_square[2, 0] + F_dec_square[3, 0])))))
    N_Intermediate[2] = (math.sqrt((1/Mu) * math.sqrt((math.sqrt(F_dec_square[4, 0] + F_dec_square[5, 0])))))
    N_Intermediate[3] = (math.sqrt((1/Mu) * math.sqrt((math.sqrt(F_dec_square[6, 0] + F_dec_square[7, 0])))))
    N_Intermediate[4] = (math.sqrt((1/Mu) * math.sqrt((math.sqrt(F_dec_square[8, 0] + F_dec_square[9, 0])))))
    N_Intermediate[5] = (math.sqrt((1/Mu) * math.sqrt((math.sqrt(F_dec_square[10, 0] + F_dec_square[11, 0])))))

    #print("N_intermediate")
    #print(N_Intermediate)

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

    N_Combined = (0.5*N_Intermediate)

    # Angles of Tilt_Rotors
    i = 0
    for i in range(6):
        Tilt[i] = math.atan2(F_dec[i+1, 0], F_dec[i, 0])

    # Giving Angular Velocity and Tilt Angle to Respective Rotor
    t_dash = 0
    if (t_dash == 0):
        speed.angular_velocities.append(N_Combined[4])
        speed.angular_velocities.append(N_Combined[1])
        speed.angular_velocities.append(N_Combined[0])
        speed.angular_velocities.append(N_Combined[3])
        speed.angular_velocities.append(N_Combined[5])
        speed.angular_velocities.append(N_Combined[2])
        speed.angular_velocities.append(N_Combined[4])
        speed.angular_velocities.append(N_Combined[1])
        speed.angular_velocities.append(N_Combined[0])
        speed.angular_velocities.append(N_Combined[3])
        speed.angular_velocities.append(N_Combined[5])
        speed.angular_velocities.append(N_Combined[2])
        speed.angular_velocities.append(Tilt[4])
        speed.angular_velocities.append(Tilt[1])
        speed.angular_velocities.append(Tilt[0])
        speed.angular_velocities.append(Tilt[3])
        speed.angular_velocities.append(Tilt[5])
        speed.angular_velocities.append(Tilt[2])
        t_dash += 1
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
    
    return(speed)   