#!/bin/bash
for i in 1 2 3 4 5; do 
	echo $i.jpg
done;

for i in `seq 1 5`; do 
	echo $i.jpg
done;

for i in {1..5}; do 
	echo $i.jpg
done;
