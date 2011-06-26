#!/usr/bin/python

from PIL import Image, ImageFont, ImageDraw
import colorsys

im = Image.new('RGB', (100,200), (255,0,0))
dr = ImageDraw.Draw(im)

dr.line(((0,0),(10,10)), fill="black")

im.save("line.png")

#draw thick line
dr.line(((0,0),(10,10)), fill="black", width=10)
im.show()
