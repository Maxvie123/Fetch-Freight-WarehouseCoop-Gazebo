#!/usr/bin/env python

# Updated: 2020-2-10
# This script is used to allocate items to picker robots. It will generate csv files which
# contain picker and tansporter destination list.


import random
import numpy as np
import sys
from parseCSVstring import *

#Example: python task_allocation.py 10 3 2

if len(sys.argv) == 4:
	item_qty = int(sys.argv[1])
	picker_qty = int(sys.argv[2])
	trans_qty = int(sys.argv[3])
else:
	print("please provide 2 paramters:item quantity and picker quantity")
	quit()

random.seed(100)
item_list = random.sample(xrange(0, 320), item_qty)

picker_matrix = np.linspace(0,0,item_qty*picker_qty).reshape(picker_qty,item_qty).astype(int)
trans_matrix = np.linspace(0,0,item_qty*trans_qty).reshape(trans_qty,item_qty).astype(int)

for i in range(picker_qty):
	picker_initial = random.choice(item_list)
	item_list.remove(picker_initial)
	picker_matrix[i,0] = picker_initial

trans_matrix[:,0] = random.sample(picker_matrix[:,0],trans_qty)

Distance_Matrix_File = '../testdata/distance.csv'
rawData = parseCSVstring(Distance_Matrix_File, returnJagged=True, fillerValue=-1, delimiter=',', commentChar='%')

# The list of expected travel distance for each picker [picker1,picker2,...,pickern]
picker_expect = np.linspace(0,0,picker_qty)
trans_expect = np.linspace(0,0,trans_qty)

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

flag = 0
picker_matrix[:,flag].asarray()


# f = picker_matrix.astype(int)
print trans_matrix
print remain



