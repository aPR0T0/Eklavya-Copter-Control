#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from mav_msgs.msg import Actuators
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
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

#calculating position for the controller
def calPosition(pos):
    global x,y,z
    x = round(pos.pose[1].position.x,3)
    y = round(pos.pose[1].position.y,3)
    z = round(pos.pose[1].position.z,3)
    # rospy.loginfo("\nX = {0}\nY = {1}\nZ = {2}".format(x,y,z))#for debugging purposes
#here I have eliminated the gps sensor because it is not installed on the firefly
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
            speed_publisher.angular_velocities[0] = 546.35
            speed_publisher.angular_velocities[1] = 546.35
            speed_publisher.angular_velocities[2] = 546.35
            speed_publisher.angular_velocities[3] = 546.35
            speed_publisher.angular_velocities[4] = 546.35
            speed_publisher.angular_velocities[5] = 546.35
        rospy.loginfo(speed_publisher)
        rospy.Subscriber("/firefly/ground_speed", TwistStamped,calVelocity)
        rospy.Subscriber("/firefly/gps", NavSatFix,calAltitude)
        rospy.Subscriber("/firefly/imu", Imu, calImu)
        rospy.Subscriber("/gazebo/model_states",ModelStates,calPosition)
        pub.publish(speed_publisher)
        i+=1
        rate.sleep()

if __name__ == '__main__':
    try:
        takeoff()
    except rospy.ROSInterruptException:
        pass