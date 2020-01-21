#! /usr/bin/env python

import actionlib
import rospy
import std_msgs.msg

from math import sin, cos, sqrt
from geometry_msgs.msg import PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from warehousetest.srv import *

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self, robot_name):
    	robot_namespace = robot_name+"/move_base"
        self.client = actionlib.SimpleActionClient(robot_namespace, MoveBaseAction)
        rospy.loginfo(robot_name+" is waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo(robot_name+" successfully connected to the server")
        service_name = robot_name+'/get_goal'
        rospy.wait_for_service(service_name)

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

# def get_current_goal(data,goal_2d):
#     goal_2d[0] = data.position.x
#     goal_2d[1] = data.position.y
#     move_base.goto(goal_2d[0], goal_2d[1], 0)

def get_goal_2d_client(counter,robot_name):

    try:
        get_goal_2d = rospy.ServiceProxy(robot_name+'/get_goal', GetGoal)
        goal_2d = get_goal_2d(counter)
        return goal_2d.goal_2d
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def get_state(msg):

    global self_state
    self_state = msg.data
    return

def get_counter(msg):

    global counter
    counter = msg.data
    return


if __name__ == "__main__":

    picker_list = [1,1,0]
    # Create a node
    counter = 0

    rospy.init_node("navigation_demo", anonymous=True)
    node_name = rospy.get_name()
    node_namespace = rospy.get_namespace()

    if node_namespace == '/':
        robot_name = rospy.get_param(node_name + '/robot_name')
    else:
        robot_name = rospy.get_param(node_namespace + node_name + '/robot_name')

    # print "{}".format(robot_name)

    # create the needed subscribers and service clients 
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    state_sub = rospy.Subscriber(robot_name + '/state', std_msgs.msg.Float32, get_state)
    counter_sub = rospy.Subscriber(robot_name + '/counter',std_msgs.msg.Int16, get_counter)
    # for picker_id in range(picker_num):
    #     rospy.Subscriber('fetch'+str(picker_id+1)+'/state',std_msgs.msg.Float32, get_picker_state, picker_id)

    rospy.sleep(2)
    #Get model position in Gazebo
    model = GetModelStateRequest()
    model.model_name=robot_name
    model.relative_entity_name='ground_plane'

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup action client
    move_base = MoveBaseClient(robot_name)


    # main loop
    while counter < len(picker_list):
        if self_state == 0:
            rospy.sleep(2)
            goal = get_goal_2d_client(counter,robot_name)
            print "moving to {}".format(goal)
            move_base.goto(goal[0], goal[1], goal[2])
        


    # goto_temp finish flag
    # flag = 0.8
    # move_base.goto_temp(0, 1, 90,model,flag)
    # move_base.goto_temp(-4, 1, 180,model,flag)


