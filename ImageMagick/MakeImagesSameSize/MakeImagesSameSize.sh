#!/bin/bash
#http://www.imagemagick.org/Usage/resize/

#try to make images the same size, but keep aspect ratio (ie. output images are not necessarily 100x100)
#for model in {1..5}; do 
#	File="${model}.jpg"
#	convert $File -resize 100x100 $File
#done;


#make images the same size, but stretch them
#for model in {1..5}; do 
#	File="${model}.jpg"
#	convert $File -resize 100x100\! $File
#done;

#make images the same size, but pad them so they are not streched
#for model in {1..5}; do 
#	File="${model}.jpg"
#	convert $File -resize 100x100 -gravity center -extent 100x100 $File
#done;

#make images the same size, but pad them so they are not streched, and change the pad color
for model in {1..5}; do 
	File="${model}.jpg"
	convert $File -resize 100x100 -background black -gravity center -extent 100x100 $File
done;
