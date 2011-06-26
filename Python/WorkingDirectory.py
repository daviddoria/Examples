#!/usr/bin/python

import os

#get working directory
print os.getcwd()

#change working directory
path = "/home/"
os.chdir(path)

#list files in current directory
os.listdir(".")
