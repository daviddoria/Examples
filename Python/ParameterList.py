#!/usr/bin/python
import sys
from glob import glob
from optparse import OptionParser

parser = OptionParser()
parser.add_option("-i", "--integer", dest="MyInt", default=3,type="int",
                  help="an integer")
parser.add_option("-f", "--float", dest="MyFloat", default=5.6,type="float",
                  help="a float")
parser.add_option("-t", "--triple", nargs=3,dest="MyTriple", type="float",
                  help="a triple")
parser.add_option("--files", 
                 help="a list of files")
(options, args) = parser.parse_args()

filelist=glob(options.files)

print options.MyInt
print options.MyFloat
print options.MyTriple
print options.files

print "FileList:"
print filelist
