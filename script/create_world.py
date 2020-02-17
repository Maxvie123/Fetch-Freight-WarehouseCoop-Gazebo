#!/usr/bin/env python

############################################################################################################################
# This script will convert a .csv file of x,y coordinates into a .world file containing racks located at each x,y coordinate.
# Also generate the corresponding pgm map file and map configuration yaml file.
# Also generate the position data in map frame for pickers and transporters.
# Author: Jingwei Liu
# Version 1.1
# Date: 02/11/2020

# USAGE:
# cd ~/catkin_ws/src/warehousetest/scripts
# python create_world.py
##############################################################################################################################


from parseCSVstring import *
from pgm import *
import sys
import numpy as np
import math

# Assign variable values
if len(sys.argv) == 7:
	rack_l = float(sys.argv[1])
	rack_w = float(sys.argv[2])
	map_l = float(sys.argv[3])
	map_w = float(sys.argv[4])
	res = float(sys.argv[5])  # map resolution
	lot_qty = int(sys.argv[6])
else:
	rack_l = 4
	rack_w = 2
	map_l = 45
	map_w = 30
	res = 0.05
	slot_qty = 320

print("Rack length:{} meters; Rack width:{} meters".format(rack_l,rack_w))
print("Map length:{} meters ; Map width:{} meters".format(map_l,map_w))
print("Map resolution:{} meters/pixel".format(res))


tag_list = ['zero','one','two','three','four','five','six','seven','eight','nine']

class make_coords:
	def __init__(self, x, y, xc, yc):
		# Sets the value of myCoords[box_number]
		self.x 	= x
		self.y 	= y
		self.xc = xc
		self.yc = yc

# Initialize our data structure to store our x,y coordinates for each rack:
myCoords = {}

dataFile = '../testdata/sample_racklocation.csv'
rawData = parseCSVstring(dataFile, returnJagged=False, fillerValue=-1, delimiter=',', commentChar='%')

for i in range(4,len(rawData)):
	x = int(rawData[i][0])
	y = int(rawData[i][1])
	xc = int(rawData[i][3])
	yc = int(rawData[i][4])
	myCoords[i+1] = make_coords(x, y, xc, yc)

# Open the outfile for writing:
outFile = open("my_mesh.world",'w')

preamble = """<?xml version="1.0"?>
<sdf version="1.4">
  <world name="Warehouse">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>2000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
"""

outFile.write(preamble)

# Create each rack model and its tag on the map:
for i in myCoords:
    rackmodel = """    
    <model name="rack{}">
      <pose>{} {} 0  0 0 0</pose>
      <static>true</static>
      <link name="body">
        <visual name="visual">
          <geometry>
            <mesh><uri>file://warehouserack.dae</uri></mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <mesh><uri>file://warehouserack.dae</uri></mesh>
          </geometry>
        </collision>
      </link>
    </model>
    """.format(str(i-4), myCoords[i].x, myCoords[i].y)
    outFile.write(rackmodel)

    tag_str = str(i-4)
    if len(tag_str) == 1:
      tagmodel = """
    <model name="rack{}_tag">
      <pose>{} {} {}  0 0 0</pose>
      <static>true</static>
      <link name="body">
        <visual name="visual">
          <geometry>
            <mesh><uri>file://{file}.dae</uri></mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <mesh><uri>file://{file}.dae</uri></mesh>
          </geometry>
        </collision>
      </link>
    </model>
    """.format(tag_str,myCoords[i].x,myCoords[i].y-2,2.4,file = tag_list[i-4])
    else:
      tens_digit = int(tag_str[0])
      unit_digit = int(tag_str[1])
      tagmodel = """
    <model name="rack{No}_tag">
      <pose>{x} {} {z}  0 0 0</pose>
      <static>true</static>
      <link name="body">
        <visual name="visual">
          <geometry>
            <mesh><uri>file://{file1}.dae</uri></mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <mesh><uri>file://{file1}.dae</uri></mesh>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="rack{No}_tag">
      <pose>{x} {} {z}  0 0 0</pose>
      <static>true</static>
      <link name="body">
        <visual name="visual">
          <geometry>
            <mesh><uri>file://{file2}.dae</uri></mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <mesh><uri>file://{file2}.dae</uri></mesh>
          </geometry>
        </collision>
      </link>
    </model>
    """.format(myCoords[i].y-1, myCoords[i].y-3, No=tag_str, x=myCoords[i].x, z = 2.4, file1= tag_list[tens_digit], file2= tag_list[unit_digit])  
    outFile.write(tagmodel)

#finish the world file
closing = """ 
  </world>
</sdf>
"""
outFile.write(closing)

outFile.close()

print("Finish creating 'my_mesh.world', please copy and paste the world file in the 'worlds' folder.")


# Create the pgm map
map_file = "warehouse.pgm"
map_array = np.linspace(255,255,540000).reshape(600,900)

for i in myCoords:
	low_x = int((myCoords[i].x - rack_w/2)/res)
	hi_x = int((myCoords[i].x + rack_w/2)/res+1)
	low_y = int((map_w - myCoords[i].y)/res)
	hi_y = int((map_w - myCoords[i].y + rack_l)/res+1)

	map_array[low_y:hi_y,low_x:hi_x] = 0

pgmwrite(map_array,map_file)

# Create the associate yaml file
outFile1 = open("warehouse.yaml",'w')

content = """
image: warehouse.pgm
resolution: {}
origin: [{}, {}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""".format(res,0,0)

outFile1.write(content)
outFile1.close()

print("Finish creating 'warehouse.pgm' and 'warehouse.yaml', please copy and paste these two files in the maps folder")


# Generate the picker and transporter position in the map frame
picker_slot_x = np.linspace(0,0,slot_qty).tolist()
picker_slot_y = np.linspace(0,0,slot_qty).tolist()
trans_slot_x = np.linspace(0,0,slot_qty).tolist()
trans_slot_y = np.linspace(0,0,slot_qty).tolist()

for i in range(slot_qty):
  rack_id = int(math.floor(i/8)+1)
  rack_center_x = myCoords[rack_id+4].xc
  rack_center_y = myCoords[rack_id+4].yc
  slot_id = int(i % 8)
  if slot_id < 4:
    picker_slot_x[i] = rack_center_x - 1.5
    picker_slot_y[i] = rack_center_y +(slot_id-1.5)
    trans_slot_x[i] = rack_center_x - 2.5
    trans_slot_y[i] = rack_center_y 
  else:
    picker_slot_x[i] = rack_center_x + 1.5
    picker_slot_y[i] = rack_center_y +(slot_id-5.5)
    trans_slot_x[i] = rack_center_x + 2.5
    trans_slot_y[i] = rack_center_y

# store the position data in csv files
with open('../testdata/picker_slot_x.csv','w') as csvfile:
	writer = csv.writer(csvfile)
	writer.writerow(picker_slot_x)

with open('../testdata/picker_slot_y.csv','w') as csvfile:
	writer = csv.writer(csvfile)
	writer.writerow(picker_slot_y)

with open('../testdata/trans_slot_x.csv','w') as csvfile:
	writer = csv.writer(csvfile)
	writer.writerow(trans_slot_x)

with open('../testdata/trans_slot_y.csv','w') as csvfile:
	writer = csv.writer(csvfile)
	writer.writerow(trans_slot_y)

print("Finish creating picker and transporter position corresponding in the map frame. Please check those files(***_slot_x.csv, ***_slot_y.csv) in the testdata folder.")