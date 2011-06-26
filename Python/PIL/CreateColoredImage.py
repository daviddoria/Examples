#!/usr/bin/python

from PIL import Image
import colorsys

im = Image.new('RGB', (100,200), (255,0,0))
im.save("test.png")

c = colorsys.hsv_to_rgb(0,1,1) # h (0 - 1), s (0 - 1), v ( 0 - 1)
im2 = Image.new('RGB', (100,200), (int(255*c[0]), int(255*c[1]), int(255*c[2])))
im2.save("testHSV.png")


c = colorsys.hls_to_rgb(0,1,1) #note HLS not HSL
im2 = Image.new('RGB', (100,200), (int(255*c[0]), int(255*c[1]), int(255*c[2])))
im2.save("testHSV.png")

