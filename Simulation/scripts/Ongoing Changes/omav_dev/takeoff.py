#!/usr/bin/env python3
# license removed for brevity
import rospy
import message_filters
from rospy.topics import Publisher
from std_msgs.msg import Float64, Float64MultiArray
from mav_msgs.msg import Actuators  
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

altitude = 0
"""
# Gets current velocity  of drone from gps_vel sensor
def calVelocity(msg):
    global vel_x, vel_y, vel_z
    vel_x = msg.twist.linear.x
    vel_y = msg.twist.linear.y
    vel_z = msg.twist.linear.z
    rospy.loginfo("\nvel_x = {0}\nvel_y = {1}\nvel_z = {2}\n".format(vel_x,vel_y,vel_z))
#calculating the altitude from the gps sensor
def calAltitude(msg):
    global altitude
    altitude  = msg.altitude
    rospy.loginfo("\nAltitude = " + str(altitude))

#calculating roll pitch for controller
def calImu(msg):
    orinetation_list = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    global roll, pitch, yaw 
    (roll,pitch,yaw) = euler_from_quaternion(orinetation_list)
    roll = roll * (180/3.14159265)
    pitch = pitch * (180/3.14159265)
    yaw = yaw * (180/3.14159265)
    # rospy.loginfo("\nRoll = {0}\nPitch = {1}\nYaw = {2}\n".format(roll,pitch,yaw))  
"""
#calculating position for the controller
def getOdo(msg):

    global time, secs, nsecs
    secs = msg.header.stamp.secs
    nsecs = (msg.header.stamp.nsecs)
    time = secs + (nsecs/1000000000)
    #rospy.loginfo("Time = {0}".format(time))

    global x, y, altitude
    x = round(msg.pose.pose.position.x, 3)
    y = round(msg.pose.pose.position.y, 3)
    altitude = round(msg.pose.pose.position.z, 3) 
    #rospy.loginfo("X: {0}, Y: {1} & Altitude: {2}".format(x, y, altitude))

    orinetation_list = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
    global roll, pitch, yaw 
    (roll,pitch,yaw) = euler_from_quaternion(orinetation_list)
    roll = roll * (180/3.14159265)
    pitch = pitch * (180/3.14159265)
    yaw = yaw * (180/3.14159265)
    #rospy.loginfo("\nRoll = {0}\nPitch = {1}\nYaw = {2}\n".format(roll,pitch,yaw))
    # rospy.loginfo("\nX = {0}\nY = {1}\nZ = {2}".format(x,y,z))#for debugging purposes
#here I have eliminated the gps sensor because it is not installed on the firefly


r = 6.8
r1 = 5
a1 = 0
a2 = 1

def takeoff():
    pub = rospy.Publisher('/omav/command/motor_speed', Actuators, queue_size=10)
    rospy.init_node('takeoff_node', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    #so by experimenting with some values we finally have a 546.5 as better just takeoff speed of the rotors
    speed_publisher = Actuators()
    speed_publisher.angular_velocities.append(100*r)
    speed_publisher.angular_velocities.append(100*r)
    speed_publisher.angular_velocities.append(100*r)
    speed_publisher.angular_velocities.append(100*r)
    speed_publisher.angular_velocities.append(100*r)
    speed_publisher.angular_velocities.append(100*r)
    speed_publisher.angular_velocities.append(100*r)
    speed_publisher.angular_velocities.append(100*r)
    speed_publisher.angular_velocities.append(100*r)
    speed_publisher.angular_velocities.append(100*r)
    speed_publisher.angular_velocities.append(100*r)
    speed_publisher.angular_velocities.append(100*r)





    speed_publisher.angular_velocities.append(1*a1)
    speed_publisher.angular_velocities.append(1*a1)
    speed_publisher.angular_velocities.append(1*a1)
    speed_publisher.angular_velocities.append(1*a1)
    speed_publisher.angular_velocities.append(1*a1)
    speed_publisher.angular_velocities.append(1*a1)



    i = 0
    while not rospy.is_shutdown():



        rospy.loginfo(speed_publisher)
        rospy.Subscriber("/omav/odometry_sensor1/odometry", Odometry, getOdo)
        print(altitude)
        alt_err_pub = rospy.Publisher("/alt_err", Float64, queue_size=10)
        alt_err_pub.publish(altitude)
        pub.publish(speed_publisher)
        i+=1
        rate.sleep()

if __name__ == '__main__':
    try:
        takeoff()
    except rospy.ROSInterruptException:
        pass