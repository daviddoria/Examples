#!/bin/bash
for i in 1 2 3 4 5; do 
	NewNumber=`printf %04d "$i"`
	echo $i $NewNumber
done;
