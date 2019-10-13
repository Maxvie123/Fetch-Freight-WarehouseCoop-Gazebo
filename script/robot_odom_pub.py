#! /usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

#initial the node
rospy.init_node('odom_pub')

#get the robot model name from the parent 
robot_name=rospy.get_param('robot_name')

#publish the odom infomation in parent namespace
odom_pub=rospy.Publisher ('odom', Odometry,queue_size=50)

#create the tf broadcaster
odom_broadcaster = tf.TransformBroadcaster()


# wait for the 'gazebo/get_model_state' service launch
rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

#get the odometry from Gazebo
odom=Odometry()
header = Header()
header.frame_id=robot_name +'/odom'
odom.child_frame_id = robot_name +'_tf/base_link'

model = GetModelStateRequest()
model.model_name=robot_name
model.relative_entity_name='ground_plane'

r = rospy.Rate(100)

tf_parent = robot_name +'_tf/odom'
tf_child = robot_name +'_tf/base_link'

while not rospy.is_shutdown():
	current_time = rospy.Time.now()

	result = get_model_srv(model)

	tf_pose_x = result.pose.position.x
	tf_pose_y = result.pose.position.y
	tf_pose_z = result.pose.position.z
	tf_head_x = result.pose.orientation.x
	tf_head_y = result.pose.orientation.y
	tf_head_z = result.pose.orientation.z
	tf_head_w = result.pose.orientation.w

	tf_pose = (tf_pose_x, tf_pose_y, tf_pose_z)
	tf_head = (tf_head_x, tf_head_y, tf_head_z, tf_head_w)

    #publish transform over tf
	odom_broadcaster.sendTransform(tf_pose,tf_head,current_time,tf_child,tf_parent)

    

   #publish odometry message over ROS
	odom.pose.pose = result.pose
	odom.twist.twist = result.twist

	header.stamp = rospy.Time.now()
	odom.header = header

	odom_pub.publish(odom)

	r.sleep()