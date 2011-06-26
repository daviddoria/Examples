#!/usr/bin/python

from PIL import Image, ImageFont, ImageDraw, ImageFile
import colorsys

backgroundim = Image.open("bkgd.png")
dimensions = backgroundim.size
w = dimensions[0]
h = dimensions[1]

bottomrightim = Image.open("BottomRight.png")
neww=200
newh=200
#bottomrightim.thumbnail((neww, newh))
bottomrightim.thumbnail((neww, newh), Image.ANTIALIAS)
print "BR: " + str(bottomrightim.size)

#allow a pad of 10 pixels on the edge of the image
#topleft = (w - neww - 10, h-newh -10)
#bottomright = (w-10, h-10)
#b = (topleft[0], topleft[1], bottomright[0], bottomright[1])
#backgroundim.paste(bottomrightim, b)

print "BBOX: " + str(bottomrightim.getbbox())
b = bottomrightim.getbbox()
print str(b)
pad = 10
newTLx = b[0] + w - b[2] - pad
newTLy = b[1] + h - b[3] - pad
newb = (newTLx, newTLy, newTLx + b[2], newTLy + b[3])
print str(newb)

backgroundim.paste(bottomrightim, newb)
backgroundim.save("overlay.png")


