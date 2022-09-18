
from cmath import cos, sin, sqrt
import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

g  = 9.81
kq = 4 #> 4
kr = 4 #> 4
t1 = 0.866025404
def force_desired(phi, theta, gamma, Mu, kap, len, t1, mass_total, prop_pos_mat, diff_pose_mat, i_pose_mat):
    #problem may occur so better use arrays
    #rotational matrix ->> We need this to transform  
    Rot_Matrix = np.array([[cos(theta)*cos(gamma),sin(gamma)*cos(theta),-sin(phi)],[sin(phi)*sin(theta)*cos(gamma)-cos(phi)*sin(gamma),sin(phi)*sin(theta)*sin(gamma)+cos(phi)*cos(gamma),sin(phi)*cos(theta)],[cos(phi)*sin(theta)*cos(gamma)+sin(phi)*sin(gamma),cos(phi)*sin(theta)*sin(gamma)-sin(phi)*cos(gamma),cos(phi)*cos(theta)]])#for body to earth

    Rot_Matrix = np.transpose(Rot_Matrix) #for body to earth
    t1 = round(0.866025404,3)

    #allocation matrix ->> We need to find its transpose and then its pseudo inverse
    #<___possibility 1___># here the sines and cos are interchanged
    A = np.array([[0,0.5,0,1,0,0.5,0,-0.5,0,-1,0,-0.5],[0,t1,0,0,0,-t1,0,-t1,0,0,0,t1],[-1,0,-1,0,-1,0,-1,0,-1,0,-1,0],[len*0.5,kap*0.5*(1/Mu),len,-kap*(1/Mu),len*0.5,kap*0.5*(1/Mu),-len*0.5,0.5*kap*(1/Mu),-len,-kap*(1/Mu),-len*0.5,kap*0.5*(1/Mu)],[t1*len,t1*kap*(1/Mu),0,0,-t1*len,-t1*kap*(1/Mu),-t1*len,t1*kap*(1/Mu),0,0,t1*len,-t1*kap*(1/Mu)],[len*0.5,-0.5*kap*(1/Mu),len,kap*(1/Mu),0.5*len,-0.5*kap*(1/Mu),0.5*len,0.5*kap*(1/Mu),len,-kap*(1/Mu),0.5*len,0.5*kap*(1/Mu)]]) #confirmed
    A = np.round_(A,decimals=2)
    #Transpose of A
    A_trans = np.round_(np.transpose(A),decimals=2)
# <--------------------------------pseudo inverse------------------------------>
    X = np.round_(np.array(np.matmul(A,A_trans)),decimals=2)
    # m <= n for A therefore, A(pseudo) =  A^T * ( A * A^T ) ^ -1
# Now, for the pseudo inverse we need X^-1s
    X_inv = np.round_(np.linalg.inv(X),decimals=2)

    A_pseudo_inv = np.matmul(A_trans,X_inv)# Now, we have the pseudo inverse ready for the given matrix

    # Gravitational matrix
    grav_matrix = np.array([[0],[0],[-g]])
    # The below given matrix is the result of total F-des without its rotation 
    res_matrix = ( mass_total*grav_matrix -  prop_pos_mat - diff_pose_mat - i_pose_mat ) #this is from earths frame so we need it in the body frame
    res_matrix  = np.round_(res_matrix,decimals=2)
    # F_desired calculation
    # print(res_matrix)

    F_des =  np.matmul( Rot_Matrix , res_matrix )
    F_des = np.round_(F_des.real , decimals = 2)
    
    # if (F_des[0][0] < 0.0000001) : F_des[0][0] = 0 
    # if (F_des[1][0] < 0.0000001) : F_des[1][0] = 0 
    # if (F_des[2][0] < 0.0000001) : F_des[2][0] = 0 
    # print(F_des[2][0])

    return F_des, A_pseudo_inv
    # So, now we have 3x1 force vector



