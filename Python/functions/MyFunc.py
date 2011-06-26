#!/usr/bin/python

def MyFunc(n):
	s = 0
	for i in range(1,n):
		s = s + i

	return s

#call with
# import MyFunc
# MyFunc.MyFunc(4)

#or

# from MyFunc import *
# MyFunc(4)
