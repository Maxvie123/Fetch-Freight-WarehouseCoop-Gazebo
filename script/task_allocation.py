#!/usr/bin/env python

##############################################################################################
# This script is used to allocate items to picker robots. It will generate csv files which
# contain picker and tansporter destination list.
# Author: Jingwei Liu
# Version 1.0
# Jingwei Liu

# USAGE:
#Example: python task_allocation.py 10 3 2 320
###############################################################################################

import random
import numpy as np
import sys
from parseCSVstring import *
import pandas as pd


if len(sys.argv) == 5:
	item_qty = int(sys.argv[1])
	picker_qty = int(sys.argv[2])
	trans_qty = int(sys.argv[3])
	slot_qty = int(sys.argv[4])
else:
	print("please provide 4 paramters:item quantity, picker quantity, transporter quantity, slots quantity")
	quit()


def list_type_converter(l,dtype=float):
	return list(map(dtype,l))

def list_to_dataframe(l):
	return pd.DataFrame({'data':l})

# set seeds
random.seed(19)
# generate item list need to be picked
item_list = random.sample(xrange(0, slot_qty), item_qty)

# matrices that store the items that each picker and transporter should pick
picker_matrix = np.linspace(0,0,item_qty*picker_qty).reshape(picker_qty,item_qty).astype(int)
trans_matrix = np.linspace(0,0,item_qty*trans_qty).reshape(trans_qty,item_qty).astype(int)


# slot index is from 0 to slot_qty-1.
for i in range(picker_qty):
	picker_initial = random.choice(item_list)
	item_list.remove(picker_initial)
	picker_matrix[i,0] = picker_initial


# get the warehouse distrance matrix
Distance_Matrix_File = '../testdata/distance.csv'
rawData = parseCSVstring(Distance_Matrix_File, returnJagged=True, fillerValue=-1, delimiter=',', commentChar='%')

# The list of expected travel distance for each picker [picker1,picker2,...,pickern] and each transporter [trans1,trans2,...transn]
picker_expect = np.linspace(0,0,picker_qty)
trans_expect = np.linspace(0,0,trans_qty)


# build item list for each picker
while (len(item_list)>0):
	minimal = float('inf')
	# find the next item and its picker
	for i in item_list:
		for j in range(picker_qty):
			current = np.max(np.nonzero(picker_matrix[j,:]))
			start = picker_matrix[j,current]
			new_tra = float(rawData[i][start])+ float(picker_expect[j])
			if new_tra < minimal:
				minimal = new_tra
				item = i
				picker = j

	# remove the item from item list
	item_list.remove(item)

	# updata expected travel distance and picker destination list
	picker_expect[picker] = minimal	
	current = np.max(np.nonzero(picker_matrix[picker,:]))
	picker_matrix[picker,current+1] = item


# build item list for each transporter
flag = 0
t_list = picker_matrix[:,flag].tolist()

while np.count_nonzero(t_list) != 0 or len(t_list) != picker_qty:
	minimal = float('inf')
	for i in t_list:
		for j in range(trans_qty):
			if np.count_nonzero(trans_matrix[j,:]) == 0:
				current = 0
			else:
				current = np.max(np.nonzero(trans_matrix[j,:]))
			start = trans_matrix[j,current]
			new_tra = float(rawData[i][start])+ float(trans_expect[j])
			if new_tra < minimal:
				minimal = new_tra
				item = i
				trans = j

	# updata expected travel distance and trans destination list
	trans_expect[trans] = minimal	
	if np.count_nonzero(trans_matrix[trans,:]) == 0:
		current = -1
	else:
		current = np.max(np.nonzero(trans_matrix[trans,:]))
	trans_matrix[trans,current+1] = item
	
	# remove the item from t_list
	t_list.remove(item)
	# update flag and t_list
	if 	np.count_nonzero(t_list) == 0:
		flag = flag + 1
		t_list = picker_matrix[:,flag].tolist()

# get the position data in the map frame for picker and transporter into dataframes
rawData = parseCSVstring('../testdata/picker_slot_x.csv', returnJagged=True, fillerValue=-1, delimiter=',', commentChar='%')
picker_slot_x = list_type_converter(rawData[0])
psx_df = list_to_dataframe(picker_slot_x)
rawData = parseCSVstring('../testdata/picker_slot_y.csv', returnJagged=True, fillerValue=-1, delimiter=',', commentChar='%')
picker_slot_y = list_type_converter(rawData[0])
psy_df = list_to_dataframe(picker_slot_y)
rawData = parseCSVstring('../testdata/trans_slot_x.csv', returnJagged=True, fillerValue=-1, delimiter=',', commentChar='%')
trans_slot_x = list_type_converter(rawData[0])
tsx_df = list_to_dataframe(trans_slot_x)
rawData = parseCSVstring('../testdata/trans_slot_y.csv', returnJagged=True, fillerValue=-1, delimiter=',', commentChar='%')
trans_slot_y = list_type_converter(rawData[0])
tsy_df = list_to_dataframe(trans_slot_y)

# generate the csv files for each picker
for i in range(picker_qty):
	df = pd.DataFrame([])
	p_list = picker_matrix[i,:][np.nonzero(picker_matrix[i,:])].tolist()
	colname1 = 'fetch'+str(i+1)+'_x'
	colname2 = 'fetch'+str(i+1)+'_y'
	colname3 = 'fetch'+str(i+1)+'_partner'
	colname4 = 'fetch'+str(i+1)+'_yaw'
	df[colname1] = psx_df['data'].ix[p_list]
	df[colname2] = psy_df['data'].ix[p_list]
	counter = 0
	partner_list = []
	yaw_list = []
	for j in p_list:
		result = np.where(trans_matrix == j)
		partner_list.append(result[0][0]+1)
		if (j%8) < 4:
			yaw_list.append(0)
		else:
			yaw_list.append(180)
		counter += 1
	df[colname3] = partner_list
	df[colname4] = yaw_list
	filename = '../testdata/fetch'+str(i+1)+'.csv'
	df.to_csv(filename,index=False)

# generate the csv files for each transporter
for i in range(trans_qty):
	df = pd.DataFrame([])
	t_list = trans_matrix[i,:][np.nonzero(trans_matrix[i,:])].tolist()
	colname1 = 'freight'+str(i+1)+'_x'
	colname2 = 'freight'+str(i+1)+'_y'
	colname3 = 'freight'+str(i+1)+'_partner'
	colname4 = 'freight'+str(i+1)+'_yaw'	
	df[colname1] = tsx_df['data'].ix[t_list]
	df[colname2] = tsy_df['data'].ix[t_list]
	counter = 0
	partner_list = []
	yaw_list = []
	for j in t_list:
		result = np.where(picker_matrix == j)
		partner_list.append(result[0][0]+1)
		yaw_list.append(0)
		counter +=1
	df[colname3] = partner_list
	df[colname4] = yaw_list
	filename = '../testdata/freight'+str(i+1)+'.csv'
	df.to_csv(filename,index=False)