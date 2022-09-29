import numpy as np
import math
from std_msgs.msg import Float64MultiArray,Float64
t1 = 0.866025404
def force_dec(F_desired, M_desired, Mu, kap, len, flag):
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

    # Astatic 
    A_static = np.array([[0,0.5*Mu,0,1*Mu,0,0.5*Mu,0,-0.5*Mu,0,-1*Mu,0,-0.5*Mu],[0,t1*Mu,0,0,0,-t1*Mu,0,-t1*Mu,0,0,0,t1*Mu],[-Mu,0,-Mu,0,-Mu,0,-Mu,0,-Mu,0,-Mu,0],[len*0.5*Mu,kap*0.5,len*Mu,-kap,len*0.5*Mu,kap*0.5,-len*0.5*Mu,0.5*kap,-len*Mu,-kap,-len*0.5*Mu,kap*0.5],[t1*len*Mu,t1*kap,0,0,-t1*len*Mu,-t1*kap,-t1*len*Mu,t1*kap,0,0,t1*len*Mu,-t1*kap],[len*0.5*Mu,-0.5*kap,len*Mu,kap,0.5*len*Mu,-0.5*kap,0.5*len*Mu,0.5*kap,len*Mu,-kap,0.5*len*Mu,0.5*kap]])
    if (flag != 0):
        # To Get Pseudo-Inverse of Astaic
        A_transpose = np.transpose(A_static)
        A_inverse_intermediate = np.matmul(A_transpose, A_static)
        A_inverse = np.linalg.inv(A_inverse_intermediate)
        A_pseudo_inverse = np.matmul(A_inverse, A_transpose)

        # Desired Matrix
        desired[0][0] = F_desired[0][0]
        desired[1][0] = F_desired[1][0]
        desired[2][0] = F_desired[2][0]
        desired[3][0] = M_desired[0][0]
        desired[4][0] = M_desired[1][0]
        desired[5][0] = M_desired[2][0]

        #F_dec Calculation
        F_dec = np.matmul(A_pseudo_inverse, desired)

    return(F_dec)

