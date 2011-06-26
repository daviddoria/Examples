#!/usr/bin/python
import os

from optparse import OptionParser

parser = OptionParser()
parser.add_option("--files", nargs=2, type="string",
                 help="a list of files")
(options, args) = parser.parse_args()

#print options.files
for CurrentFile in options.files:
	print CurrentFile
	#print os.path.basename(CurrentFile)
	CurrentName = os.path.splitext(CurrentFile)
	print CurrentName[0]
	print CurrentName[1]

#change extension
for CurrentFile in options.files:
	print CurrentFile
	CurrentName = os.path.splitext(CurrentFile)
	NewFile = CurrentName[0] + ".jpg"
	print NewFile
