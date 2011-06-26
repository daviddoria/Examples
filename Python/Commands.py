#!/usr/bin/python

import os
import sys
from glob import glob
from optparse import OptionParser

parser = OptionParser()
parser.add_option("--files", 
                 help="a list of files")
(options, args) = parser.parse_args()

filelist=glob(options.files)
print filelist

for FILE in filelist:
	print "FILE: " + FILE
	CurrentFile = os.path.split(FILE)
	print "CurrentFile: " + CurrentFile[0] + " + " + CurrentFile[1]
	CurrentName = os.path.splitext(CurrentFile[1])
	print "CurrentName: " + CurrentName[0] + " + " + CurrentName[1]
	NewFile = CurrentName[0] + "big.png"
	print "Converting " + FILE + " to " + NewFile
	cmd = "convert -scale 200% " + FILE + " " + NewFile
	os.system(cmd)

