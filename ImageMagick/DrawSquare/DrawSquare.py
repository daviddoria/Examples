#!/usr/bin/python
import os, sys

#RGB
#cmd = "convert -size 200x100 xc:rgb\(255,0,0\) square.png"

#HSL
#Saturation and lightness are represented as percentages
#100% is full saturation, and 0% is a shade of grey
#0% lightness is black, 100% lightness is white, and 50% lightness is 'normal'

#hsl(0, 100%,  50%)    red
#hsl(120, 100%,  50%)  green
#hsl(120, 100%,  25%)  light green
#hsl(120, 100%,  75%)  dark green
#hsl(120, 50%,  50%)   pastel green

cmd = "convert -size 200x100 xc:hsl\(100,100\%, 50\%\) square.png"
os.system(cmd)


