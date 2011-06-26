#!/usr/bin/python

from PIL import Image

im = Image.open("bkgd.png")
w = im.size[0] 
h = im.size[1]

print "W = " + str(w)
print "H = " + str(h)
