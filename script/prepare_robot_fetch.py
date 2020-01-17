#!/usr/bin/env python

import sys

import rospy
import time
import actionlib
from std_msgs.msg import Float64
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

arm_intermediate_positions  = [1.32, 0, -1.4, 1.72, 0.0, 1.66, 0.0]
arm_joint_positions  = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

head_joint_names = ["head_pan_joint", "head_tilt_joint"]
head_joint_positions = [0.0, 0.0]


if __name__ == "__main__":

    rospy.sleep(2)

    # initialize the node
    rospy.init_node("prepare_simulated_robot")

    # connect to the action server
    rospy.loginfo("Waiting for arm_controller...")
    arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("...connected.")

    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    rospy.loginfo("...connected.")
    rospy.loginfo("Waiting for head_controller...")

    trajectory = JointTrajectory()
    trajectory.joint_names = head_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = head_joint_positions
    trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(1.0)

    head_goal = FollowJointTrajectoryGoal()
    head_goal.trajectory = trajectory
    head_goal.goal_time_tolerance = rospy.Duration(0.0)

    trajectory = JointTrajectory()
    trajectory.joint_names = arm_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = [0.0] * len(arm_joint_positions)
    trajectory.points[0].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(1.0)
    # trajectory.points.append(JointTrajectoryPoint())
    # trajectory.points[1].positions = arm_intermediate_positions
    # trajectory.points[1].velocities =  [0.0] * len(arm_joint_positions)
    # trajectory.points[1].accelerations = [0.0] * len(arm_joint_positions)
    # trajectory.points[1].time_from_start = rospy.Duration(4.0)
    # trajectory.points.append(JointTrajectoryPoint())
    # trajectory.points[2].positions = arm_joint_positions
    # trajectory.points[2].velocities =  [0.0] * len(arm_joint_positions)
    # trajectory.points[2].accelerations = [0.0] * len(arm_joint_positions)
    # trajectory.points[2].time_from_start = rospy.Duration(7.5)

    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = trajectory
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)



    rospy.loginfo("Setting positions...")
    arm_client.send_goal(arm_goal)
    head_client.send_goal(head_goal)
    arm_client.wait_for_result(rospy.Duration(6.0))
    head_client.wait_for_result(rospy.Duration(6.0))
    rospy.loginfo("...done")

    robot_name=rospy.get_param('robot_name')

    pub1 = rospy.Publisher('/' + robot_name +'/l_gripper_finger_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/' + robot_name +'/r_gripper_finger_controller/command', Float64, queue_size=10)

    l_gripper_finger_data = 0.04
    r_gripper_finger_data = 0.04

    start_time = rospy.Time.from_sec(time.time())
    end_time = start_time + rospy.Duration(3)

    while (end_time > rospy.Time.from_sec(time.time())):

        pub1.publish(l_gripper_finger_data)
        pub2.publish(r_gripper_finger_data)