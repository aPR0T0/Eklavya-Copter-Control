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
        F_dec = np.zeros((24, 1))
        A_static = np.zeros((6, 24))
        A_transpose = np.zeros((24, 6))
        A_inverse_intermediate = np.zeros((24, 24))
        A_inverse = np.zeros((24, 24))
        A_pseudo_inverse = np.zeros((24, 6))
        desired = np.zeros((6, 1))

    # Astatic - Chinese Paper - omitting arm_length in (1*5)term
    """
    A[0] = [-1, 0, 1, 0, 0.5, 0, -0.5, 0, -0.5, 0, 0.5, 0]
    A[1] = [0, 0, 0, 0, t1, 0, -t1, 0, t1, 0, -t1, 0]
    A[2] = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
    A[3] = [-kap, -len, -kap, len, 0.5*kap, 0.5*len, 0.5*kap, -0.5*len, 0.5*kap, -0.5*len, 0.5*kap, 0.5*len]
    A[4] = [0, 0, 0, 0, -t1*kap, -t1*len, -t1*kap, t1*len, t1*kap, -t1*len, t1*kap, t1*len]
    A[5] = [-len, kap, -len, kap, -len, kap, -len, -kap, -len, -kap, -len, kap]
    """
    t1 = math.cos((30*math.pi)/180)
    #print(t1)
    kap = kappa/Mu
    len = arm_length

    A_static[0] = [-1, 0, 1, 0, 0.5, 0, -0.5, 0, -0.5, 0, 0.5, 0, -1, 0, 1, 0, 0.5, 0, -0.5, 0, -0.5, 0, 0.5, 0]
    A_static[1] = [0, 0, 0, 0, t1, 0, -t1, 0, t1, 0, -t1, 0, 0, 0, 0, 0, t1, 0, -t1, 0, t1, 0, -t1, 0]
    A_static[2] = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
    A_static[3] = [-kap, -len, -kap, len, 0.5*kap, 0.5*len, 0.5*kap, -0.5*len, 0.5*kap, -0.5*len, 0.5*kap, 0.5*len, -kap, -len, -kap, len, 0.5*kap, 0.5*len, 0.5*kap, -0.5*len, 0.5*kap, -0.5*len, 0.5*kap, 0.5*len]
    A_static[4] = [0, 0, 0, 0, -t1*kap, -t1*len, -t1*kap, t1*len, t1*kap, -t1*len, t1*kap, t1*len, 0, 0, 0, 0, -t1*kap, -t1*len, -t1*kap, t1*len, t1*kap, -t1*len, t1*kap, t1*len]
    A_static[5] = [-len, kap, -len, -kap, -len, kap, -len, -kap, -len, -kap, -len, kap, -len, kap, -len, -kap, -len, kap, -len, -kap, -len, -kap, -len, kap]
    
    A_static = Mu*A_static

    if (flag != 0):
        # To Get Pseudo-Inverse of Astaic
        A_transpose = np.transpose(A_static)
        
        A_inverse_intermediate = np.matmul(A_static, A_transpose)
        A_inverse = np.linalg.inv(A_inverse_intermediate)
        A_pseudo_inverse = np.matmul(A_transpose, A_inverse)

        # Desired Matrix
        desired[0, 0] = F_desired[0, 0]
        desired[1, 0] = F_desired[1, 0]
        desired[2, 0] = F_desired[2, 0]
        desired[3, 0] = M_desired[0, 0]
        desired[4, 0] = M_desired[1, 0]
        desired[5, 0] = M_desired[2, 0]
        #print(desired)
        #F_dec Calculation
        #print(desired)
        F_dec = np.matmul(A_pseudo_inverse, desired)
        #print(A_pseudo_inverse)

    return(F_dec)

