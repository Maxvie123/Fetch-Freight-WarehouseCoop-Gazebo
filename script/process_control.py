#!/usr/bin/env python


##############################################################################
# This file will take the initial Fetch and Freight pose list as input.
# The purpose of this file is to correctly publish current pose infomation to 
# the correspoding robot and publish robot states.
# Author: Jingwei Liu
# Version 1.0
# Date: 01/06/2020
############################################################################


import rospy
import rospkg
import sys
import pandas as pd
import geometry_msgs.msg
import actionlib_msgs.msg
import std_msgs.msg
from warehousetest.srv import GetGoal, GetGoalResponse
from warehousetest.srv import GetGripperGoal, GetGripperGoalResponse


picker_state = []
trans_state = []


# return action status
def return_status(data):

	global status

	if data.status_list == []:
		status = ""
		return status
	else:
		status_size = len(data.status_list)
		status = data.status_list[status_size-1].status
		return status

def return_gripper_status(data):

	global g_status

	if data.status_list == []:
		g_status = 0
		return g_status
	else:
		status_size = len(data.status_list)
		g_status = data.status_list[status_size-1].status
		if g_status == 4:
			g_status = 3
		return g_status



def handle_get_gripper_goal(req):

	return GetGripperGoalResponse([req.x, req.y, req.z])



class freight_process():

	global picker_num		# picker quantity
	global picker_state 	# picker state
	global picker_partner	# which picker associated with current transporter(freight)
	picker_num = 4
	picker_state = [0]*picker_num	# create all zero list which lisr length equal to pciker_num
	picker_index = range(1,picker_num+1)
	picker_partner = [-1]*picker_num

	def __init__(self,robotname):

		rospy.init_node(robotname+'_process_control')
		rospy.Service(robotname+'/get_goal', GetGoal, self.handle_get_goal)
		rospy.Subscriber(robotname+'/move_base/status', actionlib_msgs.msg.GoalStatusArray, return_status, queue_size = 1)
		self.state_pub = rospy.Publisher(robotname+'/state', std_msgs.msg.Float32, queue_size = 1)
		self.counter_pub = rospy.Publisher(robotname+'/counter', std_msgs.msg.Int16, queue_size = 1)
		self.partner_pub = rospy.Publisher(robotname+'/partner', std_msgs.msg.Int16, queue_size = 1)
		for picker_id in (range(picker_num)):
			rospy.Subscriber('fetch'+str(picker_id+1)+'/state', std_msgs.msg.Float32, self.get_picker_state, freight_process.picker_index[picker_id])
			rospy.Subscriber('fetch'+str(picker_id+1)+'/partner', std_msgs.msg.Int16, self.get_partner, freight_process.picker_index[picker_id])
		# misc varibles 
		self.robotname = robotname
		self.state = 0
		self.start = 0
		self.battery = 100
		self.listsize = len(robot_list)
		self.counter = 0
		self.rate = rospy.Rate(5)


	def handle_get_goal(self, req):
		p = req.goal_index
		if p == self.counter:
			print "Get request for goal No.{}.".format(p)
			return GetGoalResponse([robot1_x_list[p], robot1_y_list[p], robot1_yaw_list[p]])



	def get_picker_state(self, msg, picker_id):

		picker_state[picker_id-1] = round(msg.data,1)
		# print "picker {} state: {}".format(picker_id, picker_state[picker_id-1])
		return	

	def get_partner(self, msg, picker_id):

		picker_partner[picker_id-1] = msg.data
		return


	def state_control(self):

 		global status

 		# status = ""

 		counter = self.counter
 		my_id = int(self.robotname[7])


 		while (self.start == 1):
			if status == "":
				self.state = 0

			if (self.state == 0 and status == 1):
				self.state = 1
				rospy.loginfo(self.robotname+" is moving to goal position")
			elif (self.state == 1 and status == 3):
				self.state = 2
				rospy.loginfo(self.robotname+" reached the goal position")

			if (self.state == 2 and robot_list[counter] == 0):
				self.state = 0
				rospy.loginfo(self.robotname+" is idle (ready for new task)")
				counter = counter + 1
				rospy.sleep(2)
			elif (self.state == 2 and robot_list[counter] != 0):
				self.state = 3
				picker_id = robot_list[counter]
				rospy.loginfo(self.robotname+" is waiting for the items")

			if (self.state == 3 and picker_state[picker_id-1]== 4.5 and my_id == picker_partner[picker_id-1]):
				self.state = 4
				rospy.loginfo(self.robotname+" got the items from picker")

			if (self.state == 4):
				counter = counter + 1
				rospy.sleep(5)
				self.state = 0
				rospy.loginfo(self.robotname+" is idle (ready for new task)")

			if counter < len(robot1_x_list):
				self.counter = counter
				self.state_pub.publish(self.state)
				self.counter_pub.publish(self.counter)
				self.partner_pub.publish(robot_list[self.counter])

			self.rate.sleep()




class fetch_process():

	global trans_num		# transporter quantity
	global trans_state		# transporter state
	global trans_partner	# which transporter associated with current picker(fetch)
	trans_num = 3
	trans_state = [0]*trans_num	# create all zero list which lisr length equal to trans_num
	trans_index = range(1,trans_num+1)
	trans_partner = [-1]*trans_num

	# initialize the node and create two services for 2d navigation goal and gripper goal. create a subscriber for 
	def __init__(self,robotname):

		rospy.init_node(robotname+'_process_control')
		rospy.Service(robotname+'/get_goal', GetGoal, self.handle_get_goal)
		rospy.Service(robotname+'/get_gripper_goal', GetGripperGoal, handle_get_gripper_goal)
		rospy.Subscriber(robotname+'/move_base/status', actionlib_msgs.msg.GoalStatusArray, return_status, queue_size = 1)
		rospy.Subscriber(robotname+'/move_group/status', actionlib_msgs.msg.GoalStatusArray, return_gripper_status, queue_size = 1)
		self.state_pub = rospy.Publisher(robotname+'/state', std_msgs.msg.Float32, queue_size = 1)
		self.counter_pub = rospy.Publisher(robotname+'/counter', std_msgs.msg.Int16, queue_size = 1)
		self.partner_pub = rospy.Publisher(robotname+'/partner', std_msgs.msg.Int16, queue_size = 1)
		for trans_id in (range(trans_num)):
			rospy.Subscriber('freight'+str(trans_id+1)+'/state', std_msgs.msg.Float32, self.get_trans_state, fetch_process.trans_index[trans_id])
			rospy.Subscriber('freight'+str(trans_id+1)+'/partner', std_msgs.msg.Int16, self.get_partner, fetch_process.trans_index[trans_id])

		# misc varibles 
		self.robotname = robotname
		self.state = 0
		self.start = 0
		self.battery = 100
		self.listsize = len(robot_list)
		self.counter = 0
		self.rate = rospy.Rate(5)	


	def handle_get_goal(self, req):
		p = req.goal_index
		if p == self.counter:
			print "Get request for goal No.{}.".format(p)
			return GetGoalResponse([robot1_x_list[p], robot1_y_list[p], robot1_yaw_list[p]])


	def get_trans_state(self, msg, trans_id):

		trans_state[trans_id-1] = round(msg.data,1)
		# print "transporter {} state: {}".format(trans_id, trans_state[trans_id-1])
		return

	def get_partner(self, msg, trans_id):

		trans_partner[trans_id-1] = msg.data
		return


	def state_control(self):

 		global status
 		global g_status

 		# status = ""

 		counter = self.counter
 		my_id = int(self.robotname[5])


 		while (self.start == 1):
			if status == "":
				self.state = 0

			if (self.state == 0 and status == 1):
				self.state = 1
				rospy.loginfo(self.robotname+" is moving to goal position")
			elif (self.state == 1 and status == 3):
				rospy.sleep(2)
				self.state = 2
				rospy.loginfo(self.robotname+" reached the goal position")

			if (self.state == 2 and robot_list[counter] == 0):
				self.state = 0
				rospy.loginfo(self.robotname+" is idle (ready for new task)")
				counter = counter + 1
				rospy.sleep(10)
			elif (self.state == 2 and robot_list[counter] != 0):
				trans_id = robot_list[counter]
				self.state = 3

			if (self.state == 3 and g_status == 1):
				self.state = 3.1
				rospy.loginfo(self.robotname+" is picking the items")
			elif (self.state == 3.1 and g_status == 3):
				rospy.sleep(2)
				self.state = 3.2
				rospy.loginfo(self.robotname+" got the item")
			elif (self.state == 3.2 and g_status == 1):
				rospy.sleep(2)
				self.state = 3.3
				rospy.loginfo(self.robotname+" back to default arm position")
			elif (self.state == 3.3 and g_status == 3 and trans_state[trans_id-1] == 3):
				rospy.sleep(2)
				self.state = 3.4
				rospy.loginfo(self.robotname+" is ready to place item to trans")
			elif (self.state == 3.4 and my_id == trans_partner[trans_id-1] and trans_state[trans_id-1] == 3):
				self.state = 4


			if (self.state == 4 and status == 1):
				self.state = 4.1
				rospy.loginfo(self.robotname+" is moving to trans")
			elif (self.state == 4.1 and status == 3):
				rospy.sleep(2)
				self.state = 4.2
				rospy.loginfo(self.robotname+" reach cooperation position")
			elif (self.state == 4.2 and g_status == 1):
				self.state = 4.3
				rospy.loginfo(self.robotname+" is placing the item")
			elif (self.state == 4.3 and g_status == 3):
				rospy.sleep(2)
				self.state = 4.4
				rospy.loginfo(self.robotname+" has placed the item")
			elif (self.state == 4.4 and g_status == 1):
				rospy.sleep(2)
				self.state = 4.5
				rospy.loginfo(self.robotname+" back to default arm position")
			elif (self.state == 4.5 and g_status == 3):
				counter = counter + 1
				rospy.loginfo(self.robotname+" is idle (ready for new task)")
				rospy.sleep(2)
				self.state = 0

			if counter < len(robot1_x_list):
				self.counter = counter
				self.state_pub.publish(self.state)
				self.counter_pub.publish(self.counter)
				self.partner_pub.publish(robot_list[self.counter])
			
			self.rate.sleep()


if __name__ == "__main__":

	rospack = rospkg.RosPack()
	path = rospack.get_path('warehousetest')
	file_name = sys.argv[1]

	df = pd.read_csv(path+"/testdata/"+file_name+".csv")

	x_colname = file_name +"_x"
	y_colname = file_name +"_y"
	yaw_colname = file_name +"_yaw"
	partner_colname = file_name +"_partner"

	robot1_x_list = df[x_colname].tolist()
	robot1_y_list = df[y_colname].tolist()
	robot1_yaw_list = df[yaw_colname].tolist()
	robot_list = df[partner_colname].tolist()

	if "fetch" in file_name:
		robot1 = fetch_process(file_name)
	else:
		robot1 = freight_process(file_name)

	rospy.sleep(2)

	robot1.start = 1
	
	robot1.state_control()