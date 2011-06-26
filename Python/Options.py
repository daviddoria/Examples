#!/usr/bin/python
import os

from optparse import OptionParser

parser = OptionParser()
parser.add_option("--file", type="string",
                 help="a filename")
parser.add_option("--num", type="int",
                 help="a number")
(options, args) = parser.parse_args()

print options.file
print options.num
