#!/usr/bin/env python3

import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("""Reconfigure Request: {kp}, {ki}, {kd}, {kp_roll}, {ki_roll}, {kd_roll}, {kp_pitch}, {ki_pitch}, {kd_pitch},{kp_yaw}, {ki_yaw}, {kd_yaw},{kp_x}, {ki_x}, {kd_x}, {kp_y}, {ki_y}, {kd_y}, {kp_vel_x}, {ki_vel_x}, {kd_vel_x}, {kp_vel_y}, {ki_vel_y}, {kd_vel_y}""".format(**config))
    # return config

if __name__ == "__main__":
    rospy.init_node("pid_tuner")

    client = dynamic_reconfigure.client.Client("controller_node", timeout=30, config_callback=callback)

    r = rospy.Rate(1)
    kp = 20
    ki = 0.001
    kd = 35
    kp_roll = 0
    ki_roll = 0
    kd_roll = 0.005
    kp_pitch = 0
    ki_pitch = 0
    kd_pitch = 0.005
    kp_yaw = 0    
    ki_yaw = 0
    kd_yaw = 0.005
    kp_x = 0
    ki_x = 0.00001
    kd_x =  0.00015
    kp_y = 0.13
    ki_y = 0
    kd_y = 0.00015
    kp_vel_x = 0.1
    ki_vel_x = 0
    kd_vel_x = 0.071
    kp_vel_y = 0.01
    ki_vel_y = 0.0
    kd_vel_y = 0.0071
    while not rospy.is_shutdown():
        client.update_configuration({"kp":kp, "ki":ki, "kd":ki, "kp_roll":kp_roll, "ki_roll":ki_roll, "kd_roll":kd_roll, "kp_pitch":kp_pitch, "ki_pitch":ki_pitch, "kd_pitch":kd_pitch,"kp_yaw":kp_yaw, "ki_yaw":ki_yaw, "kd_yaw":kd_yaw,"kp_x":kp_x, "ki_x":ki_x, "kd_x":kd_x, "kp_y":kp_y, "ki_y":ki_y, "kd_y":kd_y, "kp_vel_x":kp_vel_x, "ki_vel_x":ki_vel_x, "kd_vel_x":kd_vel_x, "kp_vel_y":kp_vel_y, "ki_vel_y":ki_vel_y, "kd_vel_y":kd_vel_y})
        r.sleep()