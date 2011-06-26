#!/usr/bin/python
import os, sys

MyFile = open('test.txt', 'w')
#MyFile.write('test succeeded') #write a string
Temp = (2,3,4)
print Temp[0]
#MyFile.write(str(Temp))
MyFile.write(repr(Temp)) # better than str()
MyFile.write("\n")
MyFile.write(repr(Temp))
MyFile.close()

