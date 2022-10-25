#!/usr/bin/env python3
# This console is only for debugging

import rospy
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float64, Float64MultiArray
altitude = 0
s1 = 7
s  = 6.9
def setS(msg):
    s = msg[0]
    s1 = msg[1]
def takeoff():
    alt_pub  = rospy.Subscriber('s_pub', Float64MultiArray, setS)
    pub = rospy.Publisher('/omav/command/motor_speed', Actuators,queue_size=100)
    rospy.init_node('takeoff_node',anonymous=True)
    # rospy.Subscriber('/s_input',Float64MultiArray,setS)
    speed =  Actuators()
    rate = rospy.Rate(1)
    speed.angular_velocities.append(100*s)
    speed.angular_velocities.append(100*s)
    speed.angular_velocities.append(100*s)
    speed.angular_velocities.append(100*s)
    speed.angular_velocities.append(100*s)
    speed.angular_velocities.append(100*s) 
    speed.angular_velocities.append(100*s) 
    speed.angular_velocities.append(100*s)
    speed.angular_velocities.append(100*s)
    speed.angular_velocities.append(100*s)
    speed.angular_velocities.append(100*s)
    speed.angular_velocities.append(100*s)
    speed.angular_velocities.append(100*s)


    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    
    i = 0
    while not rospy.is_shutdown():
        if i == 1:
            speed.angular_velocities[0] = 100*s
            speed.angular_velocities[1] = 100*s
            speed.angular_velocities[2] = 100*s
            speed.angular_velocities[3] = 100*s
            speed.angular_velocities[4] = 100*s
            speed.angular_velocities[5] = 100*s1
            speed.angular_velocities[6] = 100*s
            speed.angular_velocities[7] = 100*s
            speed.angular_velocities[8] = 100*s
            speed.angular_velocities[9] = 100*s
            speed.angular_velocities[10] = 100*s
            speed.angular_velocities[11] = 100*s1 
            speed.angular_velocities[12] = 0
            speed.angular_velocities[13] = 0
            speed.angular_velocities[14] = 0
            speed.angular_velocities[15] = 0
            speed.angular_velocities[16] = 0
            speed.angular_velocities[17] = 0
        rospy.loginfo(speed)
        pub.publish(speed)
        i+= 1
        rate.sleep()    

if __name__=='__main__':
    try:
        takeoff()
    except rospy.exceptions.ROSInternalException:
        pass