#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import TutorialsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request:{kp}, {ki},\ 
            {kd}, {kp_x}, {ki_x}, {kd_x}, {kp_y}, {ki_y}, {kd_y}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)
    
    srv = Server(TutorialsConfig, callback)
    rospy.spin()
