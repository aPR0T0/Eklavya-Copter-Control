#from ftplib import parse150
import rospy
import message_filters
from rospy.topics import Publisher
from std_msgs.msg import Float64, Float64MultiArray
from mav_msgs.msg import Actuators  
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


def moment_desired(roll_desired, pitch_desired, yaw_desired, q_x_current, q_y_current, q_z_current, q_w_current, w_x_current, w_y_current, w_z_current):

    quaternion_desired = quaternion_from_euler(roll_desired,pitch_desired,yaw_desired)

    q_x_desired = quaternion_desired[0]
    q_y_desired = quaternion_desired[1]
    q_z_desired = quaternion_desired[2]
    q_w_desired = quaternion_desired[3]

    p0 = q_w_desired
    p1 = q_x_desired
    p2 = q_y_desired
    p3 = q_z_desired

    q0 = q_w_current
    q1 = q_x_current
    q2 = q_y_current
    q3 = q_z_current


    q_w_error = (p0*q0 - p1*q1 - p2*q2 - p3*q3)
    q_x_error = (p0*q1 + p1*q0 + p2*q3 - p3*q2)
    q_y_error = (p0*q2 - p1*q3 + p2*q0 + p3*q1)
    q_z_error = (p0*q3 + p1*q2 - p2*q1 + p3*q0)

    if(q_w_error < 0):
        sign_q_error = -1
    elif(q_w_error > 0):
        sign_q_error = 1

    q_v_error = np.matrix([[q_x_error], [q_y_error], [q_z_error]])

    w_desired = (kq * sign_q_error * q_v_error)

    w_current = np.matrix([[w_x_current], [w_y_current], [w_z_current]])

    w_error = w_desired - w_current
    
    q_intermediate_1 = (kr * w_error)

    q_intermediate_2_1 = np.matmul(J, w_current)

    q_intermediate_2_1 = np.cross(w_current, q_intermediate_2_1)

    M_desired = q_intermediate_1 + q_intermediate_2_1

    return(M_desired)





