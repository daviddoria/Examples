#!/usr/bin/python

from PIL import Image, ImageFont, ImageDraw
import colorsys

im = Image.new('RGB', (100,200), (255,0,0))
dr = ImageDraw.Draw(im)

dr.text((0,0), "test")
dr.text((0,20), "test", fill="black")
im.save("text.png")

