#!/usr/bin/env python3

import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Reconfigure Request {kp}, {ki}, {kd},{kp_x}, {ki_x}, {kd_x}, {kp_y}, {ki_y}, {kd_y}".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("pid_tuner")

    client = dynamic_reconfigure.client.Client("pid_tuner", timeout=30, config_callback=callback)

    r = rospy.Rate(0.1)
    kp = 20
    ki = 0.001
    kd = 35
    kp_x = 0
    ki_x = 0.00001
    kd_x =  0.00015
    kp_y = 0.13
    ki_y = 0
    kd_y = 0.00015
    while not rospy.is_shutdown():
        client.update_configuration({"kp":kp, "ki":ki, "kd":ki, "kp_x":kp_x, "ki_x":ki_x, "kd_x":kd_x, "kp_y":kp_y, "ki_y":ki_y, "kd_y":kd_y})
        r.sleep()