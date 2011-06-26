#!/usr/bin/python

from PIL import Image, ImageFont, ImageDraw
import colorsys

im = Image.new('RGB', (100,200), (255,0,0))
dr = ImageDraw.Draw(im)

dr.rectangle(((0,0),(10,10)), fill="black", outline = "blue")

im.save("rectangle.png")

