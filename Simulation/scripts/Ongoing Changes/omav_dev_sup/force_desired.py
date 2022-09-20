from cmath import cos, sin, sqrt
import numpy as np
g = 9.81
def force_desired( phi, theta, gamma, flag, mass_total, p_position_err, d_position_err, i_position_err):
    global rotation_matrix, F_desired
    #
    if(flag == 0):
        rotation_matrix = np.zeros((3, 3))
        d_position_err = np.zeros((3, 1))
        i_position_err = np.zeros((3, 1))
        F_desired = np.zeros((3, 1))

    #
    rotation_matrix = np.array([[cos(theta)*cos(gamma),sin(gamma)*cos(theta),-sin(phi)],[sin(phi)*sin(theta)*cos(gamma)-cos(phi)*sin(gamma),sin(phi)*sin(theta)*sin(gamma)+cos(phi)*cos(gamma),sin(phi)*cos(theta)],[cos(phi)*sin(theta)*cos(gamma)+sin(phi)*sin(gamma),cos(phi)*sin(theta)*sin(gamma)-sin(phi)*cos(gamma),cos(phi)*cos(theta)]])#for body to earth

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
    
    grav_matrix = np.array([[0],[0],[-g]])
    
    
    # The below given matrix is the result of total F-des without its rotation 
    
    F_desired = ( mass_total*grav_matrix -  p_position_err - d_position_err - i_position_err ) #this is from earths frame so we need it in the body frame
    F_desired  = np.round_(F_desired,decimals=2)
    F_desired = np.round_((np.matmul(rotation_matrix,F_desired)).real,decimals=2)
    return F_desired