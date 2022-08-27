#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from mav_msgs.msg import Actuators
from std_msgs.msg import Float64, Float64MultiArray

def takeoff():
    pub = rospy.Publisher('/firefly/command/motor_speed', Actuators, queue_size=10)
    rospy.init_node('takeoff_node', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    #so by experimenting with some values we finally have a 546.5 as better just takeoff speed of the rotors
    speed_publisher = Actuators()
    speed_publisher.angular_velocities.append(547)
    speed_publisher.angular_velocities.append(547)
    speed_publisher.angular_velocities.append(547)
    speed_publisher.angular_velocities.append(547)
    speed_publisher.angular_velocities.append(547)
    speed_publisher.angular_velocities.append(547)

    i = 0
    while not rospy.is_shutdown():
        if i == 10 :

            speed_publisher.angular_velocities[0] = 546.5
            speed_publisher.angular_velocities[1] = 546.5  
            speed_publisher.angular_velocities[2] = 546.5
            speed_publisher.angular_velocities[3] = 546.5  
            speed_publisher.angular_velocities[4] = 546.5  
            speed_publisher.angular_velocities[5] = 546.5
        rospy.loginfo(speed_publisher)
        pub.publish(speed_publisher)
        i+=1
        rate.sleep()

if __name__ == '__main__':
    try:
        takeoff()
    except rospy.ROSInterruptException:
        pass