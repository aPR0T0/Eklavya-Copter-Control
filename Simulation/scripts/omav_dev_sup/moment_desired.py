import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64MultiArray,Float64


def moment_desired(roll_desired, pitch_desired, yaw_desired, roll, pitch, yaw , w_x_current, w_y_current, w_z_current, I, kq,kr,flag):
    """
    Quaternion Error Equations : 
        qerr = qdes,IB ⊗ q̂IB =  | qw,err |
                                | qv,err | , which is a 4*1 Matrix
        where, ⊗ = Kronecker product
                qerr = Quaternion Error, which is a 4*1 Matrix
                qdes,IB = Desired Quaternion Orientation of the Body Frame to Intertial Frame, which is a 4*1 Matrix
                q̂IB = Current Quaternion Orientation of the Body Frame to Intertial Frame, which is a 4*1 Matrix
                qw,err = real part of Quaternion Error
                qv,err = vector part of Quaternion Error, which is a 3*1 Matrix

    The desired body rate ωdes (desired angular velocity) is generated from the vector part of the quaternion error qv,err as :
        ωdes = kq.sign(qw,err).qv,err
        where, . = means scalar multiplication of a vector
                ωdes = desired body rate (desired angular velocity), which is a 3*1 Matrix
                kq = tuning parameter, which is a scalar
                sign(qw,err) = the sign of the real part of the quaternion error is used to avoid the unwinding phenomena, which is a scalar
                qv,err = vector part of Quaternion Error, which is a 3*1 Matrix

    The desired moments Mdes are computed as follows :
        Mdes = kr.(ωdes - ω̂ ) - (roff × BFdes) + (ω̂  × (J.ω̂ ))
        where, . = means scalar multiplication of a vector or dot product of 2 matrices
                × = means cross product of 2 matrices
                Mdes is Desired Moments, which is a 3*1 Matrix
                kr = rate controller gain, which is a scalar
                ωdes = desired body rate (desired angular velocity), which is a 3*1 Matrix
                ω̂  = current angular velocity, which is a 3*1 Matrix
                roff = center of mass offset
                BFdes= Force Desired in x, y and z directions respectively, which is a 3*1 Matrix
                J = Inertial Matrix, which is a 3*3 Matrix
    """
    #For Initial Start to prevent Garbage Values being used or Local Variable referenced before assignment error

    # Global variables are declared to avoid their values initialized/reset to 0
    global q_w_desired, q_x_desired, q_y_desired, q_z_desired, q_w_current, q_x_current, q_y_current, q_z_current
    global p0, p1, p2, p3, q0, q1, q2, q3, q_w_error, q_x_error, q_y_error, q_z_error, sign_q_w_error
    global q_v_error, w_desired, w_error, q_intermediate_1, q_intermediate_2, q_intermediate_3_1, q_intermediate_3_2, M_desired

    #print(w_current)
    roll_desired = roll_desired * (math.pi/180)
    pitch_desired = pitch_desired * (math.pi/180)
    yaw_desired = yaw_desired * (math.pi/180)
    roll = roll
    pitch = pitch
    yaw = yaw 

    quaternion_desired = quaternion_from_euler(roll_desired,pitch_desired,yaw_desired)
    quaternion_current = quaternion_from_euler(roll, pitch, yaw)
    # INITIALIZING PARAMETERS USED IN CALCULATIONS
    if(flag == 0):
        # Quaternion Desired
        q_w_desired = 0
        q_x_desired = 0
        q_y_desired = 0
        q_z_desired = 0
        # Quaternion Current
        q_w_current = 0
        q_x_current = 0
        q_y_current = 0
        q_z_current = 0
        # Since Equations are to be made smaller_name variables used
        # Quaternion Desired x, y, z & w
        p0 = 0
        p1 = 0
        p2 = 0
        p3 = 0
        # Quaternion Current x, y, z & w
        q0 = 0
        q1 = 0
        q2 = 0
        q3 = 0
        # Quaternion Error x, y, z & w
        q_w_error = 0
        q_x_error = 0
        q_y_error = 0
        q_z_error = 0
        # Sign of Quaternion Error w
        sign_q_w_error = 0
        # 3*1 Matrix of the vector part of Quaternion Error
        q_v_error = np.zeros((3, 1))
        # 3*1 Matrix of Desired Angular Velocity
        w_desired = np.zeros((3, 1))
        # 3*1 Matrix of Error in Angular Velocity
        w_error = np.zeros((3, 1))
        # 3*1 Matrix to calculate 1st term in Moment Desired Matrix
        q_intermediate_1 = np.zeros((3, 1))
        # 3*1 Matrix to calculate 2nd term in Moment Desired Matrix
        q_intermediate_2 = np.zeros((3, 1))
        # 3*1 Matrix to calculate intermediate part of 3rd term in Moment Desired Matrix
        q_intermediate_3_1 = np.zeros((3, 1))
        # 3*1 Matrix to calculate 3rd term in Moment Desired Matrix
        q_intermediate_3_2 = np.zeros((3, 1))
        # 3*1 Matrix of Moment_Desired
        M_desired = np.zeros((3, 1))

    if (flag != 0):
        # Assigning values to Quaternion Desired
        q_x_desired = quaternion_desired[0]
        q_y_desired = quaternion_desired[1]
        q_z_desired = quaternion_desired[2]
        q_w_desired = quaternion_desired[3]

        # Assigning values to Quaternion Current
        q_x_current = quaternion_current[0]
        q_y_current = quaternion_current[1]
        q_z_current = quaternion_current[2]
        q_w_current = quaternion_current[3]

        # Assigning values to smaller_name variables
        p0 = q_w_desired
        p1 = q_x_desired
        p2 = q_y_desired
        p3 = q_z_desired

        q0 = q_w_current
        q1 = q_x_current
        q2 = q_y_current
        q3 = q_z_current

        # Quaternion Error Equations :
        q_w_error = (p0*q0 + p1*q1 + p2*q2 + p3*q3)
        q_x_error = (p0*q1 - p1*q0 + p2*q3 - p3*q2)
        q_y_error = (p0*q2 - p2*q0 - p1*q3 + p3*q1)
        q_z_error = (p0*q3 - p3*q0 + p1*q2 - p2*q1)

        """
        q0 = q_w_current
        q1 = -q_x_current
        q2 = -q_y_current
        q3 = -q_z_current

        # Quaternion Error Equations :
        q_w_error = (p0*q0 - p1*q1 - p2*q2 - p3*q3)
        q_x_error = (p0*q1 + p1*q0 + p2*q3 - p3*q2)
        q_y_error = (p0*q2 - p1*q3 + p2*q0 + p3*q1)
        q_z_error = (p0*q3 + p1*q2 - p2*q1 + p3*q0)
        """

        # We Require only the sign of q_w_error for further Calculation
        if(q_w_error < 0):
            sign_q_w_error = -1
        elif(q_w_error > 0):
            sign_q_w_error = 1

        # Initializing the 3*1 Matrix of the vector part of Quaternion Error for Further Calculations

        q_v_error = np.round_(np.array([[q_x_error], [q_y_error], [q_z_error]]),decimals=2)

        w_current = np.round_(np.array([[w_x_current], [w_y_current],[w_z_current]]),decimals=2)
        # Desired Angular Velocity :
        w_desired = (kq * sign_q_w_error * q_v_error)
        #print(w_desired)

        

        # Intermediate Calculation Terms to find Moment_Desired :

        # To Find Error in Angular Velocity which is a 3*1 Matrix
        # To Calculate : ωerror = (ωdes - ω̂ ) 
        #                ωerror = (w_current - w_current)
        w_error = w_desired - w_current
        #print(w_error)
        # To Calculate 1st Term in Moment_Desired Equation : q_intermediate_1 = kr.(ωdes - ω̂ )
        #                                                                     = kr.ωerror
        q_intermediate_1 = (kr * w_error)
        #print(q_intermediate_1)
        # To Calculate 2nd Term in Moment_Desired Equation : q_intermediate_2 = roff × BFdes
        # Cross Product
        #print(q_intermediate_2)
        # To Calculate intermediate for 3rd term in Moment_Desired Equation : q_intermediate_3_1 = J.ω̂ 
        # Dot Product
        q_intermediate_3_1 = np.round_(np.matmul(I, w_current) ,decimals=2)
        
        #print(q_intermediate_3_1)
        # To Calculate 3rd Term in Moment_Desired Equation : q_intermediate_3_2 = ω̂  × (J.ω̂ )
        #                                                                       = ω̂  × q_intermediate_3_1
        # Cross Product
        q_intermediate_3_2 = np.round_(np.cross(w_current, q_intermediate_3_1, axis=0),decimals=2)
        
        # To Calculate Moment_Desired using 1st, 2nd and 3rd Term Mdes = kr.(ωdes - ω̂ ) - (roff × BFdes) + (ω̂  × (J.ω̂ ))
        #                                                             = q_intermediate_1 - q_intermediate_2 + q_intermediate_3_2
        M_desired = q_intermediate_1 + q_intermediate_3_2
    #M_desired = np.zeros((3, 1))
    M_desired = np.round_(M_desired,decimals=2)
    return M_desired