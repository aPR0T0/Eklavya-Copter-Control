#!/usr/bin/env python3

import rospy
from mav_msgs.msg import Actuators
from std_msgs.msg import Float64, Float64MultiArray

def takeoff():
    pub = rospy.Publisher('/firefly/command/motor_speed', Actuators, queue_size=10)
    rospy.init_node('takeoff_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #so by experimenting with some values we finally have a 546.5 as better just takeoff speed of the rotors
    speed_publisher = Actuators()
    speed_publisher.angular_velocities.append(547)
    speed_publisher.angular_velocities.append(547)
    speed_publisher.angular_velocities.append(547)
    speed_publisher.angular_velocities.append(547)
    speed_publisher.angular_velocities.append(547)
    speed_publisher.angular_velocities.append(547)


    while not rospy.is_shutdown():
        rospy.loginfo(speed_publisher)
        pub.publish(speed_publisher)
        rate.sleep()


if __name__ == '__main__':
    try:
        takeoff()
    except rospy.ROSInterruptException:
        pass