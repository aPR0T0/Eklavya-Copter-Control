import numpy as np
import math
from std_msgs.msg import Float64MultiArray,Float64

def force_desired(position_desired, position_current, euler_current, current_time, sample_time, kp, kd, ki, mass, gravity, flag):

    """
    
    """
    #
    global d_time, prev_time
    global rotation_matrix, current_position_error, d_position_error, previous_position_error, i_position_error
    global i_intermediate, d_intermediate
    global p_term, i_term, d_term, gravity_term, before_transformation, F_desired

    #
    if(flag == 0):
        d_time = 0
        prev_time = 0
        rotation_matrix = np.zeros((3, 3))
        current_position_error = np.zeros((3, 1))
        d_position_error = np.zeros((3, 1))
        previous_position_error = np.zeros((3, 1))
        i_position_error = np.zeros((3, 1))
        i_intermediate = np.zeros((3, 1))
        d_intermediate = np.zeros((3, 1))

        p_term = np.zeros((3, 1))
        i_term = np.zeros((3, 1))
        d_term = np.zeros((3, 1))


        gravity_term = np.zeros((3, 1))
        before_transformation = np.zeros((3, 1))
        F_desired = np.zeros((3, 1))

    #
    d_time = current_time - prev_time

    #
    rotation_matrix[0][0] = ((math.cos(euler_current[1])) * (math.cos(euler_current[2])))
    rotation_matrix[1][0] = ((math.cos(euler_current[1])) * (math.sin(euler_current[2])))
    rotation_matrix[2][0] = (-(math.sin(euler_current[1])))
    rotation_matrix[0][1] = (((math.cos(euler_current[2])) * (math.sin(euler_current[1])) * (math.sin(euler_current[0]))) - ((math.cos(euler_current[0])) * (math.sin(euler_current[2]))))
    rotation_matrix[1][1] = (((math.cos(euler_current[0])) * (math.cos(euler_current[2]))) + ((math.sin(euler_current[1])) * (math.sin(euler_current[0])) * (math.sin(euler_current[2]))))
    rotation_matrix[2][1] = ((math.cos(euler_current[1])) * (math.sin(euler_current[0])))
    rotation_matrix[0][2] = (((math.cos(euler_current[0])) * (math.cos(euler_current[2])) * (math.sin(euler_current[1]))) + ((math.sin(euler_current[0])) * (math.sin(euler_current[2]))))
    rotation_matrix[1][2] = (((math.cos(euler_current[0])) * (math.sin(euler_current[1])) * (math.sin(euler_current[2]))) - ((math.cos(euler_current[2])) * (math.sin(euler_current[0]))))
    rotation_matrix[2][2] = ((math.cos(euler_current[1])) * (math.cos(euler_current[0])))

    if( d_time >= sample_time ):
        #
        current_position_error = position_desired - position_current
        d_position_error = current_position_error - previous_position_error
        i_position_error = (d_time * current_position_error)

        # P term
        p_term = (kp * current_position_error)

        # I term
        i_intermediate += i_position_error
        if(i_intermediate[0][0] > 800): i_intermediate[0][0] = 800
        if(i_intermediate[0][0] < -800): i_intermediate[0][0] = -800
        if(i_intermediate[1][0] > 800): i_intermediate[1][0] = 800
        if(i_intermediate[1][0] < -800): i_intermediate[1][0] = -800
        if(i_intermediate[2][0] > 800): i_intermediate[2][0] = 800
        if(i_intermediate[2][0] < -800): i_intermediate[2][0] = -800
        i_term = (ki * i_intermediate)

        #D Term
        d_intermediate = ((1/d_time) * d_position_error)
        d_term = (kd * d_intermediate)

        # Gravity term
        gravity_term = (mass * gravity)

        # Double Derivate of P_desired
        # Since we are taking Constant P_desired hence this term is 0


        # Equation
        before_transformation = p_term + i_term + d_term + gravity_term

        # F_desired
        F_desired = np.matmul(rotation_matrix, before_transformation)

        prev_time = current_time
        previous_position_error = current_position_error

    return(F_desired)