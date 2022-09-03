#!/usr/bin/env python3
# license removed for brevity
import rospy
from mav_msgs.msg import Actuators  


pub = rospy.Publisher('/omav/motor_speed', Actuators, queue_size=1000)

def takeoff():
    
    rospy.init_node('takeoff_node', anonymous=False)
    rate = rospy.Rate(20) # 10hz
    #so by experimenting with some values we finally have a 546.5 as better just takeoff speed of the rotors
    speed_publisher = Actuators()
    speed_publisher.angles.append(0)
    speed_publisher.angles.append(0)
    speed_publisher.angles.append(0)
    speed_publisher.angles.append(0)
    speed_publisher.angles.append(0)
    speed_publisher.angles.append(0)

    speed_publisher.angular_velocities.append(1600)
    speed_publisher.angular_velocities.append(1600)
    speed_publisher.angular_velocities.append(1600)
    speed_publisher.angular_velocities.append(1600)
    speed_publisher.angular_velocities.append(1600)
    speed_publisher.angular_velocities.append(1600)
    speed_publisher.angular_velocities.append(1600)
    speed_publisher.angular_velocities.append(1600)
    speed_publisher.angular_velocities.append(1600)
    speed_publisher.angular_velocities.append(1600)
    speed_publisher.angular_velocities.append(1600)
    speed_publisher.angular_velocities.append(1600)


    i = 0

    while not rospy.is_shutdown():
        if i == 10:
            speed_publisher.angles[0] = 0
            speed_publisher.angles[1] = 0
            speed_publisher.angles[2] = 0
            speed_publisher.angles[3] = 0
            speed_publisher.angles[4] = 0
            speed_publisher.angles[5] = 0

            speed_publisher.angular_velocities[0] = 500
            speed_publisher.angular_velocities[1] = 500
            speed_publisher.angular_velocities[2] = 500
            speed_publisher.angular_velocities[3] = 500
            speed_publisher.angular_velocities[4] = 500
            speed_publisher.angular_velocities[5] = 500
            speed_publisher.angular_velocities[6] = 500
            speed_publisher.angular_velocities[7] = 500
            speed_publisher.angular_velocities[8] = 500
            speed_publisher.angular_velocities[9] = 500
            speed_publisher.angular_velocities[10] = 500
            speed_publisher.angular_velocities[11] = 500

        rospy.loginfo(speed_publisher)
        i += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        takeoff()
    except rospy.ROSInterruptException:
        pass
