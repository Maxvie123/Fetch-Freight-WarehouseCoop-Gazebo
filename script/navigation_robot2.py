#! /usr/bin/env python

import actionlib
import rospy

from math import sin, cos, sqrt
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self, robot_name):
        robot_namespace = robot_name+"/move_base"
        self.client = actionlib.SimpleActionClient(robot_namespace, MoveBaseAction)
        rospy.loginfo(robot_name+" is waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo(robot_name+" successfully connected to the server")

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

    def goto_temp(self, x, y, theta, model,flag, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # Do not wait for result
        self.client.send_goal(move_goal)
        state = 0
        while not state:
            result = get_model_srv(model)
            # Get x and y coordinate values
            nowx = result.pose.position.x
            nowy = result.pose.position.y
            if sqrt((x-nowx)**2+(y-nowy)**2) < flag:
                state = 1


if __name__ == "__main__":

    # Create a node
    robot_name = "robot2"
    rospy.init_node("navigation_demo_for_"+robot_name)
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    #Get model position in Gazebo
    model = GetModelStateRequest()
    model.model_name=robot_name
    model.relative_entity_name='ground_plane'

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient("robot2")

    # goto_temp finish flag
    flag = 0.5

    rospy.sleep(5)
    rospy.loginfo("Moving to rack #5")
    # move_base.goto_temp(4, -5, -90,model,flag)
    # move_base.goto_temp(-4, -5, 180,model,flag)
    move_base.goto(-4, 1, 180)

