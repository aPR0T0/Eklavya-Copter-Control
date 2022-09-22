import numpy as np
import math
from std_msgs.msg import Float64MultiArray,Float64

def force_dec(F_desired, M_desired, Mu, kappa, arm_length, flag):

    global F_dec, A_static, A_transpose, A_inverse_intermediate, A_inverse, A_pseudo_inverse, desired


    if (flag == 0):
        F_dec = np.zeros((12, 1))
        A_static = np.zeros((6, 12))
        A_transpose = np.zeros((12, 6))
        A_inverse_intermediate = np.zeros((12, 12))
        A_inverse = np.zeros((12, 12))
        A_pseudo_inverse = np.zeros((12, 6))
        desired = np.zeros((6, 1))

    cos_30 = math.cos((30*math.pi)/180)

    """
    6*24 Allocation Matrix

    A_static[0] = [-1, 0, 1, 0, 0.5, 0, -0.5, 0, -0.5, 0, 0.5, 0, -1, 0, 1, 0, 0.5, 0, -0.5, 0, -0.5, 0, 0.5, 0]
    A_static[1] = [0, 0, 0, 0, cos_30, 0, -cos_30, 0, cos_30, 0, -cos_30, 0, 0, 0, 0, 0, cos_30, 0, -cos_30, 0, cos_30, 0, -cos_30, 0]
    A_static[2] = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
    A_static[3] = [-kappa, -arm_length, -kappa, arm_length, 0.5*kappa, 0.5*arm_length, 0.5*kappa, -0.5*arm_length, 0.5*kappa, -0.5*arm_length, 0.5*kappa, 0.5*arm_length, -kappa, -arm_length, -kappa, arm_length, 0.5*kappa, 0.5*arm_length, 0.5*kappa, -0.5*arm_length, 0.5*kappa, -0.5*arm_length, 0.5*kappa, 0.5*arm_length]
    A_static[4] = [0, 0, 0, 0, -cos_30*kappa, -cos_30*arm_length, -cos_30*kappa, cos_30*arm_length, cos_30*kappa, -cos_30*arm_length, cos_30*kappa, cos_30*arm_length, 0, 0, 0, 0, -cos_30*kappa, -cos_30*arm_length, -cos_30*kappa, cos_30*arm_length, cos_30*kappa, -cos_30*arm_length, cos_30*kappa, cos_30*arm_length]
    A_static[5] = [-arm_length, kappa, -arm_length, -kappa, -arm_length, kappa, -arm_length, -kappa, -arm_length, -kappa, -arm_length, kappa, -arm_length, kappa, -arm_length, -kappa, -arm_length, kappa, -arm_length, -kappa, -arm_length, -kappa, -arm_length, kappa]
    
    
    """

    A_static = np.array([[0,-1,0,1,0,0.5,0,-0.5,0,-0.5,0,0.5], \
                         [0,0,0,0,0,cos_30,0,-cos_30,0,cos_30,0,-cos_30], \
                         [-1,0,-1,0,-1,0,-1,0,-1,0,-1,0], \
                         [-arm_length,-kappa*(1/Mu),arm_length,-kappa*(1/Mu),arm_length*0.5,kappa*0.5*(1/Mu),-arm_length*0.5,0.5*kappa*(1/Mu),-arm_length*0.5,kappa*0.5*(1/Mu),arm_length,kappa*(1/Mu)], \
                         [0,0,0,0,cos_30*arm_length,cos_30*kappa*(1/Mu),-cos_30*arm_length,cos_30*kappa*(1/Mu),cos_30*arm_length,-cos_30*kappa*(1/Mu),-cos_30*arm_length,-kappa*cos_30*(1/Mu)], \
                         [-kappa*(1/Mu),arm_length,kappa*(1/Mu),arm_length,-kappa*(1/Mu),arm_length,kappa*(1/Mu),arm_length,kappa*(1/Mu),arm_length,-kappa*(1/Mu),arm_length]])

    A_static = Mu*A_static

    if (flag != 0):
        # To Get Pseudo-Inverse of Astaic
        A_transpose = np.transpose(A_static)
        
        A_inverse_intermediate = np.matmul(A_static, A_transpose)
        A_inverse = np.linalg.inv(A_inverse_intermediate)
        A_pseudo_inverse = np.matmul(A_transpose, A_inverse)

        # Desired Matrix
        desired[0][0] = F_desired[0][0]
        desired[1][0] = F_desired[1][0]
        desired[2][0] = F_desired[2][0]
        desired[3][0] = M_desired[0][0]
        desired[4][0] = M_desired[1][0]
        desired[5][0] = M_desired[2][0]

        F_dec = np.matmul(A_pseudo_inverse, desired)

    return(F_dec)

