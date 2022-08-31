import rospy
kp = 20
ki = 0.001
kd = 35
kp_roll = 0
ki_roll = 0
kd_roll = 0.005
kp_pitch = 0
ki_pitch = 0
kd_pitch = 0.005
kp_yaw = 0    
ki_yaw = 0
kd_yaw = 0.005
kp_x = 0
ki_x = 0.00001
kd_x =  0.00015
kp_y = 0.13
ki_y = 0
kd_y = 0.00015
kp_vel_x = 0.1
ki_vel_x = 0
kd_vel_x = 0.071
kp_vel_y = 0.01
ki_vel_y = 0.0
kd_vel_y = 0.0071
def control():
    # define global values for required parameters to avoid resetting to 0
    global altitude, thrust, speed_publisher

    # initialize node
    rospy.init_node("controller_node", anonymous = False) 
    rospy.spin()
# Main function
if __name__=='__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
        