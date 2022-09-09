from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


roll = 0
pitch = 0
yaw = 0

# Since angles will be given in degrees we need to convert to radians which is standard convention
roll = roll * (math.pi/180)
pitch = pitch * (math.pi/180)
yaw = yaw * (math.pi/180)
print(roll, pitch, yaw)
# Create a rotation object from Euler angles specifying axes of rotation
quat = quaternion_from_euler(roll,pitch,yaw)

# Convert to quaternions and print
print(quat)
