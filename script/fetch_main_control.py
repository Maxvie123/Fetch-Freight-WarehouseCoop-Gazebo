#!/usr/bin/env python

import rospy
import geometry_msgs.msg

# def compute_arm_pose():




if __name__ == "__main__":

	rospy.init_node('fetch_main_control')
	rospy.Publisher('arm_pose/goal', geometry_msgs.msg.Pose, queue_size = 10)

