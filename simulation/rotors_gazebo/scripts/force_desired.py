import math
from unicodedata import decimal
import numpy as np
g = 9.81
def force_desired( phi, theta, gamma, flag, mass_total, p_position_err, d_position_err, i_position_err, acceleration):
    global rotation_matrix, F_desired
    #
    if(flag == 0):
        rotation_matrix = np.zeros((3, 3))
        d_position_err = np.zeros((3, 1))
        i_position_err = np.zeros((3, 1))
        F_desired = np.zeros((3, 1))    

    #
    rotation_matrix = np.array([[math.cos(theta)*math.cos(gamma),math.sin(gamma)*math.cos(theta),-math.sin(phi)],
                                [math.sin(phi)*math.sin(theta)*math.cos(gamma)-math.cos(phi)*math.sin(gamma),
                                math.sin(phi)*math.sin(theta)*math.sin(gamma)+math.cos(phi)*math.cos(gamma),
                                math.sin(phi)*math.cos(theta)],
                                [math.cos(phi)*math.sin(theta)*math.cos(gamma)+math.sin(phi)*math.sin(gamma),
                                math.cos(phi)*math.sin(theta)*math.sin(gamma)-math.sin(phi)*math.cos(gamma),
                                math.cos(phi)*math.cos(theta)]]) #for body to earth

    # rotation_matrix[0][0] = cos(theta)*cos(gamma)
    # rotation_matrix[0][1] = sin(gamma)*cos(theta)
    # rotation_matrix[0][2] = -sin(phi)
    # rotation_matrix[1][0] = sin(phi)*sin(theta)*cos(gamma)-cos(phi)*sin(gamma)
    # rotation_matrix[1][1] = sin(phi)*sin(theta)*sin(gamma)+cos(phi)*cos(gamma)
    # rotation_matrix[1][2] = sin(phi)*cos(theta)
    # rotation_matrix[2][0] = cos(phi)*sin(theta)*cos(gamma)+sin(phi)*sin(gamma)
    # rotation_matrix[2][1] = cos(phi)*sin(theta)*sin(gamma)-sin(phi)*cos(gamma)
    # rotation_matrix[2][2] = cos(phi)*cos(theta)

    # rotation_matrix = np.transpose(rotation_matrix) #for body to earth
    
    # Gravitational matrix
    
    grav_matrix = np.array([[0],
                            [0],
                            [-g]])
    # print(acceleration)
    
    # The below given matrix is the result of total FN-des without its rotation 
    # F_desired = ( mass_total*grav_matrix -  p_position_err - d_position_err - i_position_err + 0.00005*mass_total*acceleration )
    F_desired = mass_total*( grav_matrix +  p_position_err + d_position_err + i_position_err - acceleration )  #this is from earths frame so we need it in the body frame
    F_desired  = np.round_(F_desired,decimals=2)
    F_desired = np.round_((np.matmul(rotation_matrix,F_desired)).real,decimals=2)
    # F_desired = F_desired - mass_total*acceleration #because acceleration is already in body frame of reference
    return F_desired