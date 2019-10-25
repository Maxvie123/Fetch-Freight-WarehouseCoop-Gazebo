#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64


#initial the node
rospy.init_node('arm_initialization')

#get the robot model name from the parent 
robot_name=rospy.get_param('robot_name')


pub1 = rospy.Publisher('/' + robot_name +'/head_pan_controller/command', Float64, queue_size=10)
pub2 = rospy.Publisher('/' + robot_name +'/head_tilt_controller/command', Float64, queue_size=10)
pub3 = rospy.Publisher('/' + robot_name +'/shoulder_pan_controller/command', Float64, queue_size=10)
pub4 = rospy.Publisher('/' + robot_name +'/shoulder_lift_controller/command', Float64, queue_size=10)
pub5 = rospy.Publisher('/' + robot_name +'/upperarm_roll_controller/command', Float64, queue_size=10)
pub6 = rospy.Publisher('/' + robot_name +'/elbow_flex_controller/command', Float64, queue_size=10)
pub7 = rospy.Publisher('/' + robot_name +'/forearm_roll_controller/command', Float64, queue_size=10)
pub8 = rospy.Publisher('/' + robot_name +'/wrist_flex_controller/command', Float64, queue_size=10)
pub9 = rospy.Publisher('/' + robot_name +'/wrist_roll_controller/command', Float64, queue_size=10)


n = 0

head_pan_data = 0.0
head_tilt_data = 0.0
shoulder_pan_data = 0.0
shoulder_lift_data = -0.5
upperarm_roll_data = 0.0
elbow_flex_data = 0
forearm_roll_data = 0.0
wrist_flex_data = 0
wrist_roll_data = 0

rospy.sleep(10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
	
	if n < 1000:
		n = n + 1
	if n < 50:
		shoulder_lift_data = -0.5
	if n > 50 and n < 100:
		shoulder_pan_data = 1.5
	if n>100 and n < 150:
		shoulder_lift_data = 1.5
		elbow_flex_data = -1.8
	if n>150 and n < 200:
		upperarm_roll_data = 1.56
		wrist_flex_data = -1.6
	if n > 220:
		upperarm_roll_data = 3.14


	pub1.publish(head_pan_data)
	pub2.publish(head_tilt_data)
	pub3.publish(shoulder_pan_data)
	pub4.publish(shoulder_lift_data)
	pub5.publish(upperarm_roll_data)
	pub6.publish(elbow_flex_data)
	pub7.publish(forearm_roll_data)
	pub8.publish(wrist_flex_data)
	pub9.publish(wrist_roll_data)
	rate.sleep()






