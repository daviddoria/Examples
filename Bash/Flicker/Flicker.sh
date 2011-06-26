#!/bin/bash
#for i in 1 2 3 4 5; do 
#	echo $i.jpg
#done;

convert -delay 20 -loop 0 *.jpg animated.gif # loop 0 means repeat infinitely
