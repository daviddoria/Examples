#!/bin/bash

#make images same size
#for model in {1..5}; do 
#	File="${model}.jpg"
#	convert $File -resize 100x100\! $File
#done;


#concatenate vertically
#convert 1.jpg 2.jpg -append append.jpg

#concatenate vertically with a border
#convert 1.jpg 2.jpg -border 10 -append append.jpg

#concatenate vertically with a blue border
#convert 1.jpg 2.jpg -bordercolor blue -border 10 -append append.jpg

#concatenate horizontally
#convert 1.jpg 2.jpg +append append.jpg

#concatenate a list of images
#convert [12345].jpg +append append.jpg #works
#convert [1,2,3,4,5].jpg +append append.jpg #works
#convert [1-5].jpg +append append.jpg #works
#convert *.jpg +append append.jpg #works
#convert 00[1-5].jpg +append append.jpg



#add a border
#montage -background blue -geometry +4+4 001.jpg test.jpg

#add a border and concatentate
#montage -background blue -geometry +4+4 001.jpg 002.jpg test.jpg


