#!/usr/bin/python

from glob import glob
FileList = glob(options.prefix + "*.py")
FileList.sort()

#access elements
print FileList[0]
print FileList[1]

