#!/bin/bash

#convert 1.jpg -geometry +100+200 test.jpg

convert 1.jpg -affine 1,0,0,0,100,200 -transform test.jpg
