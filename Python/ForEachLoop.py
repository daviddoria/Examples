#!/usr/bin/python


from optparse import OptionParser
from glob import glob

parser = OptionParser()
parser.add_option("--files", type="string",
                 help="a list of files")
(options, args) = parser.parse_args()

FileList=glob(options.files)

print "For Each:"
#print options.files
for CurrentFile in FileList:
	print CurrentFile

print "For :"
#print options.files
for i in range(0,len(FileList)):
	print FileList[i]
