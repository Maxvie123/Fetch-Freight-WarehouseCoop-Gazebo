#!/usr/bin/env python

import pandas as pd
from parseCSVstring import *
import numpy as np
from pgm import *
# picker1 = pd.read_csv("../testdata/picker1.csv")

# a = 'fetch1'

# if "fetch" in a:

# 	print"{}".format(int(a[5]))

# if int(a[5]) == 1:
# 	print"Yes" 


# dataFile = 'sample_racklocation.csv'
# rawData = parseCSVstring(dataFile, returnJagged=True, fillerValue=-1, delimiter=',', commentChar='%')
# printMatrix(rawData)

s = np.linspace(255,255,100).reshape(10,10)

s[1:5,1:5] = 0

print s 