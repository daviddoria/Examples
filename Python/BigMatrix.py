#!/usr/bin/python

import numpy
from random import *

Mats = []

#fill
for frame in range(0,5):
	#a 5x5 matrix
	M = numpy.zeros([5,5], tuple)
	for scan in range(0,5):
		for model in range(0, 5):
			p = (random(), random())
			M[model,scan] = p
			print "Frame: " + str(frame) + " model: " + str(model) + " scan: " + str(scan) + " p: " + str(p)
	Mats.append(M)


#read
for frame in range(0,5):
	for scan in range(0,5):
		for model in range(0, 5):
			p = Mats[frame][model,scan]
			print "Frame: " + str(frame) + " model: " + str(model) + " scan: " + str(scan) + " p: " + str(p)
