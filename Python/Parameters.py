#!/usr/bin/python
import sys
from glob import glob


#print all of the arguments
#for arg in sys.argv:
#	print arg

from optparse import OptionParser

parser = OptionParser()
parser.add_option("--file", dest="filename", default=True,type="string",
                  help="write report to FILE")
parser.add_option("-i", "--integer", dest="MyInt", default=3,type="int",
                  help="an integer")
parser.add_option("-f", "--float", dest="MyFloat", default=5.6,type="float",
                  help="a float")
parser.add_option("-t", "--triple", nargs=3,dest="MyTriple", type="float",
                  help="a triple")
parser.add_option("-n", "--NumFiles", type="int",
                  help="Number of files")
parser.add_option("--files", 
                 help="a list of files")
(options, args) = parser.parse_args()

print options.filename
print options.MyInt
print options.MyFloat
print options.MyTriple
#print options.MyTriple[0]
#print options.MyTriple[1]
#print options.MyTriple[2]
print options.files
