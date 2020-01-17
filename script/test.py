#!/usr/bin/env python

import sys

import rospy
import actionlib
import time
from std_msgs.msg import Float64

if __name__ == "__main__":

    # initialize the node
    rospy.init_node("testnode")

    robot_name="robot3"


    pub1 = rospy.Publisher('/' + robot_name +'/l_gripper_finger_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/' + robot_name +'/r_gripper_finger_controller/command', Float64, queue_size=10)

    l_gripper_finger_data = 0.04
    r_gripper_finger_data = 0.04

    start_time = rospy.Time.from_sec(time.time())
    end_time = start_time + rospy.Duration(3)

    print "start time ", start_time
    print "end_time", end_time

    while (end_time > rospy.Time.from_sec(time.time())):

        pub1.publish(l_gripper_finger_data)
        pub2.publish(r_gripper_finger_data)
