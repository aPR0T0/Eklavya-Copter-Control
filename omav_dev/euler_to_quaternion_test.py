from tf.transformations import euler_from_quaternion, quaternion_from_euler

orinetation_list = [1.6174642071356844e-13, 3.7624270158260115e-13, -2.3179757840648416e-20, 1.0]
(roll,pitch,yaw) = euler_from_quaternion(orinetation_list)

print(roll, pitch, yaw)
# Create a rotation object from Euler angles specifying axes of rotation
quat = quaternion_from_euler(roll,pitch,yaw)

# Convert to quaternions and print
print(quat)
