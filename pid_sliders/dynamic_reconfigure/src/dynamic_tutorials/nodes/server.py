#!/usr/bin/env python3

import imp
import rospy

from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import TutorialsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {kp}, {ki}, {kd}, {kp_roll}, {ki_roll}, {kd_roll}, {kp_pitch}, {ki_pitch}, {kd_pitch},{kp_yaw}, {ki_yaw}, {kd_yaw},{kp_x}, {ki_x}, {kd_x}, {kp_y}, {ki_y}, {kd_y}, {kp_vel_x}, {ki_vel_x}, {kd_vel_x}, {kp_vel_y}, {ki_vel_y}, {kd_vel_y}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("controller", anonymous = False)

    srv = Server(TutorialsConfig, callback)
    rospy.spin()