#!/usr/bin/python
import os, sys, glob, numpy

cmd = "convert 1.jpg -pointsize 10 -draw \"text 50,50 'Copyright' \" output.jpg"

os.system(cmd)
