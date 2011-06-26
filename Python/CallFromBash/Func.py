#!/usr/bin/python

import os
import sys
from glob import glob
import optparse

class OptionParser (optparse.OptionParser):
	def check_required (self, opt):
		option = self.get_option(opt)
		# Assumes the option's 'default' is set to None!
		if getattr(self.values, option.dest) is None:
			self.error("%s option not supplied" % option)

parser = OptionParser()
parser.add_option("--files", help="List of files")

(options, args) = parser.parse_args()

FileList=glob(options.files)

print "File list: " + str(FileList)
