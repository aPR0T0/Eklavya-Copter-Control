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

        gravity_term = np.array([[0],[0],[-9.81]])
        before_transformation = np.zeros((3, 1))
        F_desired = np.zeros((3, 1))


    difference_time = current_time - previous_time

    theta = euler_current[1]
    gamma = euler_current[2]
    phi = euler_current[0]
    rotation_matrix = np.array([[math.cos(theta)*math.cos(gamma),math.sin(gamma)*math.cos(theta),-math.sin(phi)],[math.sin(phi)*math.sin(theta)*math.cos(gamma)-math.cos(phi)*math.sin(gamma),math.sin(phi)*math.sin(theta)*math.sin(gamma)+math.cos(phi)*math.cos(gamma),math.sin(phi)*math.cos(theta)],[math.cos(phi)*math.sin(theta)*math.cos(gamma)+math.sin(phi)*math.sin(gamma),math.cos(phi)*math.sin(theta)*math.sin(gamma)-math.sin(phi)*math.cos(gamma),math.cos(phi)*math.cos(theta)]])#for body to earth
    rotation_matrix = np.round_(rotation_matrix,decimals=2)
    #rotation_matrix = np.transpose(rotation_matrix)


    if(difference_time >= sample_time):

        current_position_error = position_desired - position_current
        difference_position_error = current_position_error - previous_position_error

        # P_term
        for i in range(0, 3):
            p_position_err[i][0] = (kp[i] * current_position_error[i][0])

        # D_Term
        d_intermediate = ((1/difference_time) * difference_position_error)
        for j in range(0, 3):
            d_position_err[j][0] = (kd[j] * d_intermediate[i][0])

        # I_Term
        i_intermediate = (difference_time * current_position_error)

        for k in range(0, 19):
            for l in range(0, 3):
                i_storage[l][k] = i_storage[l][k+1]

        for k in range(0, 3):
            i_storage[k][19] = i_intermediate[k][0]

        for m in range(0, 3):
            i_sum[m][0] = np.sum(i_storage[m])

        for n in range(0, 3):
            i_position_err[n][0] = (ki[n] * i_sum[n][0])


        # Gravity term
        gravity_term = (mass * gravity)

        # Double Derivate of P_desired
        # Since we are taking Constant P_desired hence this term is 0
    previous_time = current_time
    previous_position_error = current_position_error

    # Equation
    before_transformation = gravity_term - p_position_err - i_position_err - d_position_err 

    # F_desired
    F_desired = np.matmul(rotation_matrix, before_transformation)
        
    return(np.round_(F_desired, decimals=2))

def position_error():
    global current_position_error

    return(current_position_error)