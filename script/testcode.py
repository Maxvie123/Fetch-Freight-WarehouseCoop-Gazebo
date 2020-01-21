#!/usr/bin/env python

import pandas as pd


# picker1 = pd.read_csv("../testdata/picker1.csv")

a = 'fetch1'

if "fetch" in a:

	print"{}".format(int(a[5]))

if int(a[5]) == 1:
	print"Yes" 