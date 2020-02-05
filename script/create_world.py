#!/usr/bin/env python

'''
This script will convert a .csv file of x,y coordinates into a .world file containing racks located at each x,y coordinate.
Also generate the corresponding pgm map file and map configuration yaml file

USAGE:
cd ~/catkin_ws/src/warehousetest/scripts
python create_world.py


'''

from parseCSVstring import *
from pgm import *
import sys
import numpy as np

# Assign variable values
if len(sys.argv) == 3:
	rack_l = sys.argv[1]
	rack_w = sys.argv[2]
	map_l = sys.argv[3]
	map_w = sys.argv[4]
	res = sys.argv[5]  # map resolution
else:
	rack_l = 4
	rack_w = 2
	map_l = 45
	map_w = 30
	res = 0.05

print("Rack length:{} meters; Rack width:{} meters".format(rack_l,rack_w))
print("Map length:{} meters ; Map width:{} meters".format(map_l,map_w))
print("Map resolution:{} meters/pixel".format(res))


tag_list = ['zero','one','two','three','four','five','six','seven','eight','nine']

class make_coords:
	def __init__(self, x, y):
		# Sets the value of myCoords[box_number]
		self.x 	= x
		self.y 	= y

# Initialize our data structure to store our x,y coordinates for each rack:
myCoords = {}

dataFile = 'sample_racklocation.csv'
rawData = parseCSVstring(dataFile, returnJagged=True, fillerValue=-1, delimiter=',', commentChar='%')

for i in range(4,len(rawData)):
	x = int(rawData[i][0])
	y = int(rawData[i][1])
	myCoords[i+1] = make_coords(x, y)

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
    """.format(No=tag_str,x=myCoords[i].x,myCoords[i].y-1, myCoords[i].y-3, z = 2.4, file1 )  
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