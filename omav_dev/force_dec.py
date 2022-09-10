import numpy as np
import math
from std_msgs.msg import Float64MultiArray,Float64

def force_dec(F_desired, M_desired, Mu, kappa, arm_length, flag):
    """
    
    """
    #
    global F_dec, A_static, A_transpose, A_inverse_intermediate, A_inverse, A_pseudo_inverse, desired

    #
    if (flag == 0):
        F_dec = np.zeros((12, 1))
        A_static = np.zeros((6, 12))
        A_transpose = np.zeros((12, 6))
        A_inverse_intermediate = np.zeros((12, 12))
        A_inverse = np.zeros((12, 12))
        A_pseudo_inverse = np.zeros((12, 6))
        desired = np.zeros((6, 1))

    # Astatic - Chinese Paper - omitting arm_length in (1*5)term
    A_static[0] = [(-Mu*0.5), 0, (-Mu), 0, (-Mu*arm_length*0.5), 0, (-Mu*0.5), 0, Mu, 0, (-Mu*0.5), 0]
    A_static[1] = [(-Mu*((math.sqrt(3))*0.5)), 0, 0, 0, (-Mu*((math.sqrt(3))*0.5)), 0, (-Mu*((math.sqrt(3))*0.5)), 0, 0, 0, (Mu*((math.sqrt(3))*0.5)), 0]
    A_static[2] = [0, (-Mu), 0, (-Mu), 0, (-Mu), 0, (-Mu), 0, (-Mu), 0, (-Mu)]
    A_static[3] = [(kappa*0.5), (-Mu*arm_length*0.5), (-kappa), (-Mu*arm_length), (kappa*0.5), (-Mu*arm_length*0.5), (kappa*0.5), (-Mu*arm_length*0.5), (-kappa), (Mu*arm_length), (kappa*0.5), (-Mu*arm_length*0.5)]
    A_static[4] = [(-kappa*(math.sqrt(3))), (Mu*arm_length*(math.sqrt(3))*0.5), 0, 0, (kappa*(math.sqrt(3))*0.5), (-Mu*arm_length*(math.sqrt(3))*0.5), (-kappa*(math.sqrt(3))*0.5), (-Mu*arm_length*(math.sqrt(3))*0.5), 0, 0, (kappa*(math.sqrt(3))*0.5), (Mu*arm_length*(math.sqrt(3))*0.5)]
    A_static[5] = [(Mu*arm_length), kappa, (Mu*arm_length), (-kappa), (Mu*arm_length), kappa, (Mu*arm_length), (-kappa), (Mu*arm_length), kappa, (Mu*arm_length), (-kappa)]
    
    if (flag != 0):
        # To Get Pseudo-Inverse of Astaic
        A_transpose = np.transpose(A_static)
        A_inverse_intermediate = np.matmul(A_transpose, A_static)
        A_inverse = np.linalg.inv(A_inverse_intermediate)
        A_pseudo_inverse = np.matmul(A_inverse, A_transpose)

        # Desired Matrix
        desired[0, 0] = F_desired[0, 0]
        desired[1, 0] = F_desired[1, 0]
        desired[2, 0] = F_desired[2, 0]
        desired[0, 0] = M_desired[0, 0]
        desired[1, 0] = M_desired[1, 0]
        desired[2, 0] = M_desired[2, 0]

        #F_dec Calculation
        F_dec = np.matmul(A_pseudo_inverse, desired)

    return(F_dec)

