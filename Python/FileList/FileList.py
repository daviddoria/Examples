#!/usr/bin/python

#all files in current directory
#from glob import glob
#FileList = glob("*")
#FileList.sort()

#access elements
#for item in FileList:
#	print item
#print FileList[0]
#print FileList[1]

#all files in this directory and below (recursive)
import os
import sys
fileList = []
rootdir = '.'
for root, subFolders, files in os.walk(rootdir):
    for file in files:
        fileList.append(os.path.join(root,file))
#print fileList

for item in fileList:
	print item
