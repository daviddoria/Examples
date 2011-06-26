#!/usr/bin/python
import os, sys, glob, numpy

mat1 = numpy.fromfile('matrix.txt',sep=' ')
A = numpy.reshape(mat1,(3,3))

mat2 = numpy.fromfile('matrix2.txt',sep=' ')
B = numpy.reshape(mat2,(3,3))

print str(A)
print str(B)

print str(A+B)

#access with
print str(A[0,0])
print str(A[0,1])
