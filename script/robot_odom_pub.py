#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

#initial the node
rospy.init_node('odom_pub')

#get the robot model name from the parent 
robot_name=rospy.get_param('robot_name')

#publish the odom infomation in parent namespace
odom_pub=rospy.Publisher ('odom', Odometry,queue_size=50)


# wait for the 'gazebo/get_model_state' service launch
rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom=Odometry()
header = Header()
header.frame_id=robot_name +'/odom'

model = GetModelStateRequest()
model.model_name=robot_name
model.relative_entity_name='plane'

r = rospy.Rate(10)

while not rospy.is_shutdown():
    result = get_model_srv(model)

    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish (odom)

    r.sleep()