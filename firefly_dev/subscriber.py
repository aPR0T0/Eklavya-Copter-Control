#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float64, Float64MultiArray
from gazebo_msgs.msg import ModelStates
from mav_msgs.msg import Actuators  
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from rosgraph_msgs.msg import Clock


def calImu(msg):
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
def callClock(msg):
    secs = msg.clock.secs
    nsecs = (msg.clock.nsecs)/1000000000
    time = secs + nsecs
    #print(time)

"""
    
def listener():
    rospy.init_node("listener", anonymous = False)
    rospy.Subscriber("/firefly/odometry_sensor1/odometry", Odometry, calImu)
    #rospy.Subscriber("/clock", Clock, callClock)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__=='__main__':
    listener()

    