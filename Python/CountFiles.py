#!/usr/bin/python


import glob
jpgList = glob.glob("*.jpg")
jpgCount = len(jpgList)

#or
#from glob import glob
#jpgList = glob("*.jpg")
#jpgCount = len(jpgList)

print jpgCount
