#!/bin/bash

#call with
#./Parse.sh --prefix=a.txt

for i in $*
do
	case $i in
    	--files=*)
		FILES=`echo $i | sed 's/[-a-zA-Z0-9]*=//'`
		;;
    	--searchpath=*)
		SEARCHPATH=`echo $i | sed 's/[-a-zA-Z0-9]*=//'`
		;;
    	--lib=*)
		MyLib=`echo $i | sed 's/[-a-zA-Z0-9]*=//'`
		;;
    	--default)
		DEFAULT=YES
		;;
    	*)
                # unknown option
		;;
  	esac
done

echo $FILES
echo "File 0:"
echo ${FILES[0]}

echo $SEARCHPATH
echo $MyLib
echo $DEFAULT
