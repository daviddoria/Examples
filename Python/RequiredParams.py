#!/usr/bin/python
import optparse

#call with
#./RequiredParams.py -n 12 -f test.txt
# or
#./RequiredParams.py --number 13 -f test.txt

class OptionParser (optparse.OptionParser):
	def check_required (self, opt):
		option = self.get_option(opt)
		# Assumes the option's 'default' is set to None!
		if getattr(self.values, option.dest) is None:
			self.error("%s option not supplied" % option)

parser = OptionParser()
#parser.add_option("-v", action="count", dest="verbose")
parser.add_option("-f", "--file", default=None)
parser.add_option("-n", "--number", default=None)
(options, args) = parser.parse_args()

#print "verbose:", options.verbose
print "number:", options.number
#print "n:", options.n
print "file:", options.file
parser.check_required("-f")
