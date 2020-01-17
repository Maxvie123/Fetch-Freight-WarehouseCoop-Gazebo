#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import std_msgs.msg
from warehousetest.srv import *




default_joint_value = [1.5707, 1.518, 0, 1.6, 0, 1.6, 0]


class MoveGroupPythonInteface(object):
    """MoveGroupPythonInteface"""
    def __init__(self):
      super(MoveGroupPythonInteface, self).__init__()

      ## First initialize `moveit_commander`_ and a `rospy`_ node:
      moveit_commander.roscpp_initialize(sys.argv)
      rospy.init_node('move_group_python_interface')

      ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
      ## the robot:
      robot = moveit_commander.RobotCommander()

      ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
      ## to the world surrounding the robot:
      scene = moveit_commander.PlanningSceneInterface()

      ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
      ## to one group of joints.  In this case the group is the joints in the Panda
      ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
      ## you should change this value to the name of your robot arm planning group.
      ## This interface can be used to plan and execute motions on the Panda:
      group_name = "arm_group"
      group = moveit_commander.MoveGroupCommander(group_name)

      ## We create a `DisplayTrajectory`_ publisher which is used later to publish
      ## trajectories for RViz to visualize:
      display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                     moveit_msgs.msg.DisplayTrajectory,
                                                     queue_size=20)
      ## Getting Basic Information
      ## ^^^^^^^^^^^^^^^^^^^^^^^^^
      # We can get the name of the reference frame for this robot:
      planning_frame = group.get_planning_frame()
      print "============ Reference frame: %s" % planning_frame

      # We can also print the name of the end-effector link for this group:
      eef_link = group.get_end_effector_link()
      print "============ End effector: %s" % eef_link

      # We can get a list of all the groups in the robot:
      group_names = robot.get_group_names()
      print "============ Robot Groups:", robot.get_group_names()

      # Sometimes for debugging it is useful to print the entire state of the
      # robot:
      print "============ Printing robot state"
      print robot.get_current_state()
      print ""

      # Misc variables
      self.box_name = ''
      self.robot = robot
      self.scene = scene
      self.group = group
      self.display_trajectory_publisher = display_trajectory_publisher
      self.planning_frame = planning_frame
      self.eef_link = eef_link
      self.group_names = group_names

    def go_to_joint_state(self, joint_value = default_joint_value):
      # Copy class variables to local variables to make the web tutorials more clear.
      # In practice, you should use the class variables directly unless you have a good
      # reason not to.
      group = self.group


      ## Planning to a Joint Goal
      ## ^^^^^^^^^^^^^^^^^^^^^^^^
      ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
      ## thing we want to do is move it to a slightly better configuration.
      # We can get the joint values from the group and adjust some of the values:
      joint_goal = group.get_current_joint_values()
      joint_goal[0] = joint_value[0]
      joint_goal[1] = joint_value[1]
      joint_goal[2] = joint_value[2]
      joint_goal[3] = joint_value[3]
      joint_goal[4] = joint_value[4]
      joint_goal[5] = joint_value[5]
      joint_goal[6] = joint_value[6]

      # The go command can be called with joint values, poses, or without any
      # parameters if you have already set the pose or joint target for the group
      group.go(joint_goal, wait=True)

      # Calling ``stop()`` ensures that there is no residual movement
      group.stop()

      # For testing:
      # Note that since this section of code will not be included in the tutorials
      # we use the class variable rather than the copied state variable
      # current_joints = self.group.get_current_joint_values()
      # return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose_command):
      # Copy class variables to local variables to make the web tutorials more clear.
      # In practice, you should use the class variables directly unless you have a good
      # reason not to.
      group = self.group

      ## BEGIN_SUB_TUTORIAL plan_to_pose
      ##
      ## Planning to a Pose Goal
      ## ^^^^^^^^^^^^^^^^^^^^^^^
      ## We can plan a motion for this group to a desired pose for the
      ## end-effector:
      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.orientation = pose_command.orientation
      pose_goal.position = pose_command.position
      group.set_pose_target(pose_goal)

      ## Now, we call the planner to compute the plan and execute it.
      plan = group.go(wait=True)
      # Calling `stop()` ensures that there is no residual movement
      group.stop()
      # It is always good to clear your targets after planning with poses.
      # Note: there is no equivalent function for clear_joint_value_targets()
      group.clear_pose_targets()


      # For testing:
      # Note that since this section of code will not be included in the tutorials
      # we use the class variable rather than the copied state variable
      # current_pose = self.group.get_current_pose().pose
      # return all_close(pose_goal, current_pose, 0.01)

# def get_arm_pose(data):
#   move_arm.go_to_pose_goal(data)

def get_state(msg):

  if msg.data == 3:
    try:
      get_gripper_goal = rospy.ServiceProxy('/get_gripper_goal', GetGripperGoal)
      gg = get_gripper_goal(0.5,0.5,0.5)
      gripper_pose = geometry_msgs.msg.Pose()
      gripper_pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,1)
      gripper_pose.position = geometry_msgs.msg.Point(gg.goal[0], gg.goal[1], gg.goal[2])
      move_arm.go_to_pose_goal(gripper_pose)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
  elif (msg.data == 3.2 or msg.data == 4.4):
    move_arm.go_to_joint_state()
  elif msg.data == 4.2:
    try:
      get_gripper_goal = rospy.ServiceProxy('/get_gripper_goal', GetGripperGoal)
      gg = get_gripper_goal(0.5,0.0,0.5)
      gripper_pose = geometry_msgs.msg.Pose()
      gripper_pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,1)
      gripper_pose.position = geometry_msgs.msg.Point(gg.goal[0], gg.goal[1], gg.goal[2])
      move_arm.go_to_pose_goal(gripper_pose)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

  


if __name__ == "__main__":


  move_arm = MoveGroupPythonInteface()
  move_arm.go_to_joint_state()
  pose_sub = rospy.Subscriber('state', std_msgs.msg.Float32, get_state)
  rospy.sleep(2)
  rospy.spin()



  
  # pose1.position.x = 0.7
  # pose1.position.y = 0.0
  # pose1.position.z = 0.6
  # pose1.orientation.x = 0
  # pose1.orientation.y = 0.7071
  # pose1.orientation.z = 0
  # pose1.orientation.w = 0.7071
