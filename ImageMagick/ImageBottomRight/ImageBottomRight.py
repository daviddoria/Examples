#!/usr/bin/python
import os, sys

#using gravity
cmd="composite -gravity southeast BottomRight.png bkgd.png output2.png"
os.system(cmd)

#set size and use gravity
cmd="composite \( BottomRight.png -resize 200x300 -background yellow -gravity center \) -gravity southeast bkgd.png output3.png" #it seems like the smallest dimension determines the size
os.system(cmd)
