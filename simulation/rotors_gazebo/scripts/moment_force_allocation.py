
import numpy as np
from force_desired import *
from moment_desired import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler


g  = 9.81
kq = 4 #> 4
kr = 4 #> 4
t1 = 0.8660254904
s_ccw = 1   # Counter clockwise +ve
s_cw = 1   # Clockwise -ve

def force_calc( phi, theta, gamma, Mu, kap, len, t1, mass_total, prop_pos_mat, diff_pose_mat, i_pose_mat, acceleration, flag, roll_desired, pitch_desired, yaw_desired, roll, pitch, yaw ,w_x_current, w_y_current, w_z_current, I, kq, kr):
    
    t1 = round( 0.866025404,2)

    #allocation matrix ->> We need to find its transpose and then its pseudo inverse
    #<___possibility 1___># here the sines and cos are interchanged
    
    F_des = force_desired( phi, theta, gamma, flag, mass_total, prop_pos_mat, diff_pose_mat, i_pose_mat, acceleration)
    # print(F_des)
    M_des = moment_desired(roll_desired, pitch_desired, yaw_desired, roll, pitch, yaw , w_x_current, w_y_current, w_z_current, I , kq, kr, flag)
    # print(M_des)
    A = np.array([  [   0   ,   -1  ,   0   ,   1   ,   0   ,  0.5  ,   0   , -0.5  ,   0   , -0.5  ,   0    ,  0.5  ,   0   ,   -1  ,   0   ,   1   ,   0   ,  0.5  ,   0   , -0.5  ,   0   , -0.5  ,   0    ,  0.5  ],
                    [   0   ,   0   ,   0   ,   0   ,   0   ,   t1  ,   0   ,  -t1  ,   0   ,  t1   ,   0    ,  -t1  ,   0   ,   0   ,   0   ,   0   ,   0   ,   t1  ,   0   ,  -t1  ,   0   ,  t1   ,   0    ,  -t1  ],  
                    [   -1  ,   0   ,   -1  ,   0   ,   -1  ,   0   ,   -1  ,    0  ,   -1  ,   0   ,   -1   ,  0    ,   -1  ,   0   ,   -1  ,   0   ,   -1  ,   0   ,   -1  ,    0  ,   -1  ,   0   ,   -1   ,   0   ],
                    [ -len  , -s_ccw*kap*(1/Mu) , len , -s_cw*kap*(1/Mu) , len*0.5 , s_ccw*kap*0.5*(1/Mu) , -len*0.5 , s_cw*0.5*kap*(1/Mu) , -len*0.5 , s_cw*kap*0.5*(1/Mu) , len*0.5 , s_ccw*kap*(1/Mu)*0.5 , -len  , -s_ccw*kap*(1/Mu) , len , -s_cw*kap*(1/Mu) , len*0.5 , s_ccw*kap*0.5*(1/Mu) , -len*0.5 , s_cw*0.5*kap*(1/Mu) , -len*0.5 , s_cw*kap*0.5*(1/Mu) , len*0.5 , s_ccw*kap*(1/Mu)*0.5 ],
                    [    0  ,    0  ,    0  ,   0   , t1*len , t1*s_ccw*kap*(1/Mu) , -t1*len , t1*s_cw*kap*(1/Mu) , t1*len , -t1*s_cw*kap*(1/Mu) , -t1*len , -s_ccw*kap*t1*(1/Mu) ,    0  ,    0  ,    0  ,   0   , t1*len , t1*s_ccw*kap*(1/Mu) , -t1*len , t1*s_cw*kap*(1/Mu) , t1*len , -t1*s_cw*kap*(1/Mu) , -t1*len , -s_ccw*kap*t1*(1/Mu)  ],
                    [-s_ccw*kap*(1/Mu),len,s_cw*kap*(1/Mu),len,-s_ccw*kap*(1/Mu),len,s_cw*kap*(1/Mu),len,s_cw*kap*(1/Mu),len,-s_ccw*kap*(1/Mu),len,-s_ccw*kap*(1/Mu),len,s_cw*kap*(1/Mu),len,-s_ccw*kap*(1/Mu),len,s_cw*kap*(1/Mu),len,s_cw*kap*(1/Mu),len,-s_ccw*kap*(1/Mu),len]])  #confirmed
    #Transpose of A
    A_trans = np.transpose(A)
# <--------------------------------pseudo inverse------------------------------>
    X = np.array(np.matmul(A,A_trans))
    # m <= n for A therefore, A(pseudo) =  A^T * ( A * A^T ) ^ -1
# Now, for the pseudo inverse we need X^-1s
    X_inv = np.linalg.inv(X)

    A_pseudo_inv = np.matmul(A_trans,X_inv)# Now, we have the pseudo inverse ready for the given matrix

    # F_desired calculation
    # print(res_matrix)
    A_pseudo_inv = np.round_(A_pseudo_inv , decimals = 2)

    desired = np.array([[F_des[0][0]],
                        [F_des[1][0]],
                        [F_des[2][0]],
                        [M_des[0][0]],
                        [M_des[1][0]],
                        [M_des[2][0]]]) #3x1 matrix when restrictions are applied


    F_dec =  np.matmul( A_pseudo_inv , desired )
    F_dec = np.round_(F_dec.real , decimals = 2)
    # print(F_dec)
    return F_dec



