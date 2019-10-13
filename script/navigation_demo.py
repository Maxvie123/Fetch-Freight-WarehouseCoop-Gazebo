import actionlib
import rospy

from math import sin, cos
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self, robot_name):
    	robot_namespace = robot_name+"/move_base"
        self.client = actionlib.SimpleActionClient(robot_namespace, MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

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


if __name__ == "__main__":
    # Create a node
    rospy.init_node("navigation_demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient("robot2")


    rospy.loginfo("Moving to the origin")
    move_base.goto(0, 0, 0)