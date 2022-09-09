
import time #We require Time for PID Controller


from cmath import pi


""""""

#creating publisher for the speeds of the rotors
speed_pub = rospy.Publisher("/omav/command/motor_speed", Actuators ,queue_size=100) #here we will use angles section to give angles to the tilt rotors



# We need current velocity of the model so that we know when to stop and when to go
def calVel(msg):
    global vel_x,vel_y,vel_z
    vel_x = msg.twist.twist.linear.x
    vel_y = msg.twist.twist.linear.y
    vel_z = msg.twist.twist.linear.z




    # Logging for debugging purposes
    print("\nAltitude = " + str(altitude))
    print("Required alt = ",req_alt)
    print("Roll =", roll)
    print("Pitch =", pitch)
    print("Yaw =", yaw)
    print("X = ",x)
    print("Y = ",y)

    
    # sending the data to the PID_alt function which then calculates the speed using them
    speed = PID_alt(roll, pitch, yaw, x, y, target, altitude, velocity, flag, roll_desired, pitch_desired, yaw_desired)


    speed_pub.publish(speed)
    """"""

    
    


#defining the control function to assign rotor speeds to the omav
#def control():
    #We can get literally everything we need from the odometry sensor alone, but for extreme real life case we are taking atleast two sensors to start with, so that we are less proned to errors
#    rospy.init_node('controller_node', anonymous=False)
    #So here we take readings from the IMU->Orientation and Odometry->(current_velocity & current_position) sensors
#    imu_sub = message_filters.Subscriber("/omav/ground_truth/imu", Imu)
#    odo_sub = message_filters.Subscriber("/omav/ground_truth/odometry", Odometry)
#    tr = message_filters.TimeSynchronizer([imu_sub,odo_sub],2) #2 specifies the number of messages it should take from each sensor
#    tr.registerCallback(alt_control)
#    rospy.spin()

#if __name__=='__main__':
#    try:
 #       control()
#    except rospy.ROSInterruptException:
 #       pass