
from cmath import cos, sin, sqrt
import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

g  = 9.81
kq = 4 #> 4
kr = 4 #> 4
t1 = 0.866025404
def force_desired(phi, theta, gamma, Mu, kap, len, t1, mass_total, prop_pos_mat, diff_pose_mat, i_pose_mat, kp, ki, kd):
    #problem may occur so better use arrays
    #rotational matrix ->> We need this to transform  
    Rot_Matrix = np.array([[cos(theta)*cos(gamma),sin(gamma)*cos(theta),-sin(phi)],[sin(phi)*sin(theta)*cos(gamma)-cos(phi)*sin(gamma),sin(phi)*sin(theta)*sin(gamma)+cos(phi)*cos(gamma),sin(phi)*cos(theta)],[cos(phi)*sin(theta)*cos(gamma)+sin(phi)*sin(gamma),cos(phi)*sin(theta)*sin(gamma)-sin(phi)*cos(gamma),cos(phi)*cos(theta)]])#for body to earth

    Rot_Matrix = np.transpose(Rot_Matrix) #for body to earth
    t1 = 0.866025404

    #allocation matrix ->> We need to find its transpose and then its pseudo inverse
    #<___possibility 1___># here the sines and cos are interchanged
    A = np.array([[0,0.5,0,1,0,0.5,0,-0.5,0,-1,0,-0.5],[0,t1,0,0,0,-t1,0,-t1,0,0,0,t1],[-1,0,-1,0,-1,0,-1,0,-1,0,-1,0],[len*0.5,kap*0.5*(1/Mu),len,-kap*(1/Mu),len*0.5,kap*0.5*(1/Mu),-len*0.5,0.5*kap*(1/Mu),-len,-kap*(1/Mu),-len*0.5,kap*0.5*(1/Mu)],[t1*len,t1*kap*(1/Mu),0,0,-t1*len,-t1*kap*(1/Mu),-t1*len,t1*kap*(1/Mu),0,0,t1*len,-t1*kap*(1/Mu)],[len*0.5,-0.5*kap*(1/Mu),len,kap*(1/Mu),0.5*len,-0.5*kap*(1/Mu),0.5*len,0.5*kap*(1/Mu),len,-kap*(1/Mu),0.5*len,0.5*kap*(1/Mu)]]) #confirmed

    #Transpose of A
    A_trans = np.transpose(A)

# <--------------------------------pseudo inverse------------------------------>
    X = np.array(np.matmul(A,A_trans))
    # m <= n for A therefore, A(pseudo) =  A^T * ( A * A^T ) ^ -1
# Now, for the pseudo inverse we need X^-1s
    X_inv = np.linalg.inv(X)

    A_pseudo_inv = np.matmul(A_trans,X_inv)# Now, we have the pseudo inverse ready for the given matrix

    # Gravitational matrix
    grav_matrix = np.array([[0],[0],[-g]])
    # The below given matrix is the result of total F-des without its rotation 
    res_matrix = ( mass_total*grav_matrix -  kp*prop_pos_mat - kd*diff_pose_mat - ki*i_pose_mat ) #this is from earths frame so we need it in the body frame
    # F_desired calculation
    # print(res_matrix)

    F_des =  np.matmul( Rot_Matrix , res_matrix )
    F_des = np.round_(F_des.real , decimals = 3)
    
    # if (F_des[0][0] < 0.0000001) : F_des[0][0] = 0 
    # if (F_des[1][0] < 0.0000001) : F_des[1][0] = 0 
    # if (F_des[2][0] < 0.0000001) : F_des[2][0] = 0 
    # print(F_des[2][0])

    return F_des, A_pseudo_inv
    # So, now we have 3x1 force vector

def moment_desired(roll_desired, pitch_desired, yaw_desired, roll, pitch, yaw , w_x_current, w_y_current, w_z_current, I, kq,kr):
    
    # Since angles will be given in degrees we need to convert to radians which is standard convention
    roll_desired = roll_desired * (math.pi/180)
    pitch_desired = pitch_desired * (math.pi/180)
    yaw_desired = yaw_desired * (math.pi/180)
    roll = roll
    pitch = pitch
    yaw = yaw 

    quaternion_desired = quaternion_from_euler(roll_desired,pitch_desired,yaw_desired)
    quaternion_current = quaternion_from_euler(roll, pitch, yaw)
    
    q_x_desired = quaternion_desired[0]
    q_y_desired = quaternion_desired[1]
    q_z_desired = quaternion_desired[2]
    q_w_desired = quaternion_desired[3]
    q_x_current = quaternion_current[0] 
    q_y_current = quaternion_current[1]
    q_z_current = quaternion_current[2]
    q_w_current = quaternion_current[3]

    p0 = q_w_desired
    p1 = q_x_desired
    p2 = q_y_desired
    p3 = q_z_desired

    q0 = q_w_current
    q1 = -q_x_current
    q2 = -q_y_current
    q3 = -q_z_current


    q_w_error = (p0*q0 - p1*q1 - p2*q2 - p3*q3)
    q_x_error = (p0*q1 + p1*q0 + p2*q3 - p3*q2)
    q_y_error = (p0*q2 - p1*q3 + p2*q0 + p3*q1)
    q_z_error = (p0*q3 + p1*q2 - p2*q1 + p3*q0)

    if(q_w_error < 0):
        sign_q_error = -1
    elif(q_w_error > 0):
        sign_q_error = 1

    q_v_error = np.array([[q_x_error], [q_y_error], [q_z_error]])

    w_desired = kq * sign_q_error *  q_v_error

    w_current = np.array([[w_x_current], [w_y_current],[w_z_current]])

    w_error = w_desired - w_current
    q_intermediate_1 = (kr * w_error)
    q_intermediate_2_1 = np.array(np.matmul(I, w_current))
    q_intermediate_2_1 = np.cross(w_current, q_intermediate_2_1, axis=0)
    
    M_des =  np.zeros([3,1])
    M_des = np.round_((-q_intermediate_1 + q_intermediate_2_1), decimals= 3)
    M_des = np.zeros((3,1))
    # print(M_des)
    return M_des



