# #! /usr/bin/env python3
import rospy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry

def calImu(msg):
    """
    orinetation_list = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
    global roll, pitch, yaw 
    (roll,pitch,yaw) = euler_from_quaternion(orinetation_list)
    roll = roll * (180/3.14159265)
    pitch = pitch * (180/3.14159265)
    yaw = yaw * (180/3.14159265)
    #angVel_x = msg.angular_velocity.x
    #angVel_y = msg.angular_velocity.y
    #angVel_z = msg.angular_velocity.z
    rospy.loginfo("\nRoll = {0}\nPitch = {1}\nYaw = {2}\n".format(roll,pitch,yaw))    
    #rospy.loginfo("\nWx = {0}\nWy = {1}\nWz = {2}\n".format(angVel_x,angVel_y,angVel_z))
    """
    secs = msg.header.stamp.secs
    nsecs = (msg.header.stamp.nsecs)
    time_returned = secs + (nsecs/1000000000)

    print(time_returned)
"""
def callClock(msg):
    secs = msg.clock.secs
    nsecs = (msg.clock.nsecs)/1000000000
    time = secs + nsecs
    print(time)
?"""

    
def listener():
    rospy.init_node("listener2", anonymous = False)
    rospy.Subscriber("/omav/odometry_sensor1/odometry", Odometry, calImu)
    #rospy.Subscriber("/clock", Clock, callClock)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__=='__main__':
    listener()

    