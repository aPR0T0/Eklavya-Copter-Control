import numpy as np
import math
from std_msgs.msg import Float64MultiArray,Float64

def force_desired(position_desired, position_current, euler_current, current_time, sample_time, kp, kd, ki, mass, gravity, flag):

    global difference_time, previous_time
    global rotation_matrix, current_position_error, difference_position_error, previous_position_error
    global i_intermediate, i_storage, i_sum, d_intermediate
    global p_position_err, i_position_err, d_position_err, gravity_term, before_transformation, F_desired


    if(flag == 0):
        difference_time = 0
        previous_time = current_time
        rotation_matrix = np.zeros((3, 3))
        current_position_error = np.zeros((3, 1))
        difference_position_error = np.zeros((3, 1))
        previous_position_error = np.zeros((3, 1))
        i_intermediate = np.zeros((3, 1))
        i_storage = np.zeros((3, 20))
        i_sum = np.zeros((3, 1))
        d_intermediate = np.zeros((3, 1))

        p_position_err = np.zeros((3, 1))
        i_position_err = np.zeros((3, 1))
        d_position_err = np.zeros((3, 1))

        gravity_term = np.zeros((3, 1))
        before_transformation = np.zeros((3, 1))
        F_desired = np.zeros((3, 1))



    difference_time = current_time - previous_time


    rotation_matrix[0][0] = ((math.cos(euler_current[1])) * (math.cos(euler_current[2])))
    rotation_matrix[1][0] = ((math.cos(euler_current[1])) * (math.sin(euler_current[2])))
    rotation_matrix[2][0] = (-(math.sin(euler_current[1])))
    rotation_matrix[0][1] = (((math.cos(euler_current[2])) * (math.sin(euler_current[1])) * (math.sin(euler_current[0]))) - ((math.cos(euler_current[0])) * (math.sin(euler_current[2]))))
    rotation_matrix[1][1] = (((math.cos(euler_current[0])) * (math.cos(euler_current[2]))) + ((math.sin(euler_current[1])) * (math.sin(euler_current[0])) * (math.sin(euler_current[2]))))
    rotation_matrix[2][1] = ((math.cos(euler_current[1])) * (math.sin(euler_current[0])))
    rotation_matrix[0][2] = (((math.cos(euler_current[0])) * (math.cos(euler_current[2])) * (math.sin(euler_current[1]))) + ((math.sin(euler_current[0])) * (math.sin(euler_current[2]))))
    rotation_matrix[1][2] = (((math.cos(euler_current[0])) * (math.sin(euler_current[1])) * (math.sin(euler_current[2]))) - ((math.cos(euler_current[2])) * (math.sin(euler_current[0]))))
    rotation_matrix[2][2] = ((math.cos(euler_current[1])) * (math.cos(euler_current[0])))

    rotation_matrix = np.transpose(rotation_matrix)


    if(difference_time >= sample_time):

        current_position_error = position_desired - position_current
        difference_position_error = current_position_error - previous_position_error

        # P_term
        p_position_err = np.matmul(kp, current_position_error)

        # D_Term
        d_intermediate = ((1/difference_time) * difference_position_error)
        d_position_err = np.matmul(kd, d_intermediate)

        # I_Term
        i_intermediate = (difference_time * current_position_error)

        for i in range(0, 19):
            if i<19:
                for j in range(0, 3):
                    i_storage[j][i] = i_storage[j][i+1]

        for k in range(0, 3):
            i_storage[k][19] = i_intermediate[k][0]

        for l in range(0, 3):
            i_sum[k][0] = np.sum(i_storage[k])

        i_position_err = np.matmul(ki * i_sum)


        # Gravity term
        gravity_term = (mass * gravity)

        # Double Derivate of P_desired
        # Since we are taking Constant P_desired hence this term is 0

        # Equation
        before_transformation = -p_position_err - i_position_err - d_position_err + gravity_term

        # F_desired
        F_desired = np.matmul(rotation_matrix, before_transformation)

        previous_time = current_time
        previous_position_error = current_position_error
        
        return(current_position_error, F_desired)