#!/usr/bin/python
import os, sys

MyFile = open('test.txt', 'r')
MyLine = MyFile.readline()
MyFile.close()

print MyLine
MyTuple = eval(MyLine)
print MyTuple[0]
