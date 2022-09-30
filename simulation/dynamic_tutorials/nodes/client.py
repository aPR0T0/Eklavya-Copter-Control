#!/usr/bin/env python3

import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("""Reconfigure Request: {kp}, {ki},\ 
            {kd}, {kp_x}, {ki_x}, {kd_x}, {kp_y}, {ki_y}, {kd_y}""".format(**config))

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=30, config_callback=callback)
    rospy.spin()
