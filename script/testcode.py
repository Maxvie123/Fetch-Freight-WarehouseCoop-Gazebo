#!/usr/bin/env python

import pandas as pd
from parseCSVstring import *
import numpy as np
from pgm import *
import math
import random
import csv
# picker1 = pd.read_csv("../testdata/picker1.csv")

# a = 'fetch1'

# if "fetch" in a:

# 	print"{}".format(int(a[5]))

# if int(a[5]) == 1:
# 	print"Yes" 


# dataFile = 'sample_racklocation.csv'
# rawData = parseCSVstring(dataFile, returnJagged=True, fillerValue=-1, delimiter=',', commentChar='%')
# printMatrix(rawData)


# a = np.linspace(0,0,102400).reshape(320,320)

# for i in range(0,320):
# 	for j in range(i,320):
# 		racki = int(math.floor(i/8)+1)
# 		rackj = int(math.floor(j/8)+1)
# 		sloti = i % 8
# 		slotj = j % 8
# 		racki_y = int(math.floor((racki-1)/8)+1)
# 		racki_x = racki % 8 + 1
# 		rackj_y = int(math.floor((rackj-1)/8)+1)
# 		rackj_x = rackj % 8 + 1
# 		if (sloti<4) and (slotj<4):
# 			a[i,j] = 5*abs(rackj_x-racki_x)+6*abs(rackj_y-racki_y)
# 		elif (sloti>=4) and (slotj <4):
# 			a[i,j] = 5*abs(rackj_x-racki_x)+6*abs(rackj_y-racki_y)-2
# 		elif (sloti >=4) and (slotj >=4):
# 			a[i,j] = 5*abs(rackj_x-racki_x)+6*abs(rackj_y-racki_y)
# 		else:
# 			a[i,j] = 5*abs(rackj_x-racki_x)+6*abs(rackj_y-racki_y)+2

# 		if (a[i,j] == 0) and (i!=j):
# 			a[i,j] = random.randint(1,3)

# 		a[j,i] = a[i,j]


# np.savetxt("distance.csv", a, delimiter=",")	



a = ['1','2']

r = map(int, a)

print r

