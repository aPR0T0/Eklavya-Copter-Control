#! /usr/bin/env python3
import rospy
import numpy as np
import math
import message_filters
from std_msgs.msg import Float64MultiArray,Float64
from mav_msgs.msg import Actuators
# Debugging to check whether .append is updating new values or not
#r1 = 0
#r2 = 0

def get_speed_publisher(F_dec, Mu, flag):
    """
    
    """
    #
    speed = Actuators()

    # 
    global F_dec_square, N_Intermediate, N_Combined 
    #global r1, r2

    #
    if(flag == 0):
        F_dec_square = np.zeros((24, 1))
        N_Intermediate = np.zeros(12)
        N_Combined = np.zeros(6)
        

    
    #r1 = r2*0.01

    #N_Combined = np.array([100*r1, 100*r1, 100*r1, 100*r1, 100*r1, 100*r1, 100*r1, 100*r1, 100*r1, 100*r1, 100*r1, 100*r1, 0, 0, 0, 0, 0, 0])

    #
    #print(F_dec)
    
    F_dec_square = np.square(F_dec)
    Mu = 1
    # Combined Angular Velocity at a Rotor_Point
    N_Intermediate[0] = (math.pow((F_dec_square[0][0] + F_dec_square[1][0]), 0.25))
    N_Intermediate[1] = (math.pow((F_dec_square[2][0] + F_dec_square[3][0]), 0.25))
    N_Intermediate[2] = (math.pow((F_dec_square[4][0] + F_dec_square[5][0]), 0.25))
    N_Intermediate[3] = (math.pow((F_dec_square[6][0] + F_dec_square[7][0]), 0.25))
    N_Intermediate[4] = (math.pow((F_dec_square[8][0] + F_dec_square[9][0]), 0.25))
    N_Intermediate[5] = (math.pow((F_dec_square[10][0] + F_dec_square[11][0]), 0.25))
    N_Intermediate[6] = (math.pow((F_dec_square[12][0] + F_dec_square[13][0]), 0.25))
    N_Intermediate[7] = (math.pow((F_dec_square[14][0] + F_dec_square[15][0]), 0.25))
    N_Intermediate[8] = (math.pow((F_dec_square[16][0] + F_dec_square[17][0]), 0.25))
    N_Intermediate[9] = (math.pow((F_dec_square[18][0] + F_dec_square[19][0]), 0.25))
    N_Intermediate[10] = (math.pow((F_dec_square[20][0] + F_dec_square[21][0]), 0.25))
    N_Intermediate[11] = (math.pow((F_dec_square[22][0] + F_dec_square[23][0]), 0.25))


    #print(N_Intermediate)
    if(N_Intermediate[0] > 1500): N_Intermediate[0] = 1500
    if(N_Intermediate[0] < 10): N_Intermediate[0] = 10
    if(N_Intermediate[1] > 1500): N_Intermediate[1] = 1500
    if(N_Intermediate[1] < 10): N_Intermediate[1] = 10
    if(N_Intermediate[2] > 1500): N_Intermediate[2] = 1500
    if(N_Intermediate[2] < 10): N_Intermediate[2] = 10
    if(N_Intermediate[3] > 1500): N_Intermediate[3] = 1500
    if(N_Intermediate[3] < 10): N_Intermediate[3] = 10
    if(N_Intermediate[4] > 1500): N_Intermediate[4] = 1500
    if(N_Intermediate[4] < 10): N_Intermediate[4] = 10
    if(N_Intermediate[5] > 1500): N_Intermediate[5] = 1500
    if(N_Intermediate[5] < 10): N_Intermediate[5] = 10
    if(N_Intermediate[6] > 1500): N_Intermediate[6] = 1500
    if(N_Intermediate[6] < 10): N_Intermediate[6] = 10
    if(N_Intermediate[7] > 1500): N_Intermediate[7] = 1500
    if(N_Intermediate[7] < 10): N_Intermediate[7] = 10
    if(N_Intermediate[8] > 1500): N_Intermediate[8] = 1500
    if(N_Intermediate[8] < 10): N_Intermediate[8] = 10
    if(N_Intermediate[9] > 1500): N_Intermediate[9] = 1500
    if(N_Intermediate[9] < 10): N_Intermediate[9] = 10
    if(N_Intermediate[10] > 1500): N_Intermediate[10] = 1500
    if(N_Intermediate[10] < 10): N_Intermediate[10] = 10
    if(N_Intermediate[11] > 1500): N_Intermediate[11] = 1500
    if(N_Intermediate[11] < 10): N_Intermediate[11] = 10
   
    
    # Angles of Tilt_Rotors
    N_Combined[0] = (math.atan2(((F_dec[0][0] + F_dec[12][0])), ((F_dec[1][0] + F_dec[13][0]))))
    N_Combined[1] = (math.atan2(((F_dec[2][0] + F_dec[14][0])), ((F_dec[3][0] + F_dec[15][0]))))
    N_Combined[2] = (math.atan2(((F_dec[4][0] + F_dec[16][0])), ((F_dec[5][0] + F_dec[17][0]))))
    N_Combined[3] = (math.atan2(((F_dec[6][0] + F_dec[18][0])), ((F_dec[7][0] + F_dec[19][0]))))
    N_Combined[4] = (math.atan2(((F_dec[8][0] + F_dec[20][0])), ((F_dec[9][0] + F_dec[21][0]))))
    N_Combined[5] = (math.atan2(((F_dec[10][0] + F_dec[22][0])), ((F_dec[11][0] + F_dec[23][0]))))


    N_Combined = np.round_(N_Combined, decimals = 2)
    N_Intermediate = np.round_(N_Intermediate, decimals = 2)
    # Giving Angular Velocity and Tilt Angle to Respective Rotor
    speed.angular_velocities.append(N_Intermediate[0])
    speed.angular_velocities.append(N_Intermediate[1])
    speed.angular_velocities.append(N_Intermediate[2])
    speed.angular_velocities.append(N_Intermediate[3])
    speed.angular_velocities.append(N_Intermediate[4])
    speed.angular_velocities.append(N_Intermediate[5])
    speed.angular_velocities.append(N_Intermediate[6])
    speed.angular_velocities.append(N_Intermediate[7])
    speed.angular_velocities.append(N_Intermediate[8])
    speed.angular_velocities.append(N_Intermediate[9])
    speed.angular_velocities.append(N_Intermediate[10])
    speed.angular_velocities.append(N_Intermediate[11])
    speed.angular_velocities.append(N_Combined[0])
    speed.angular_velocities.append(N_Combined[1])
    speed.angular_velocities.append(N_Combined[2])
    speed.angular_velocities.append(N_Combined[3])
    speed.angular_velocities.append(N_Combined[4])
    speed.angular_velocities.append(N_Combined[5])
    
    #r2+=1
    #print(speed)
    return(speed)