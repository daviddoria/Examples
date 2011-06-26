#!/bin/bash
ImageDir=`zenity --file-selection --directory --text="Where are the images?"`
echo "Image directory: $ImageDir"
pushd $ImageDir

FileType=`zenity --entry --text="Which type of file?"`
echo "File type selected: $FileType"

OutputFile=`zenity --entry --text="Output file name (assumed .mpg)?"`
echo "Output file selected: $OutputFile"

zenity --question --text="Do you want to use all of the images?"

if [[ $? == 0 ]] ; then #use all the images
	echo "Using all the images."
	mencoder "mf://*.$FileType" -mf type=$FileType -ovc lavc -lavcopts vcodec=huffyuv:format=422p -oac copy -o huffyuv.avi -aspect 4:3 -force-avi-aspect 1.3333334
else #select a file of which images to use
	ImageList=`zenity --file-selection --text="Which image list file to use?"`
	echo "Using image list file $ImageList"
	mencoder "mf://@$ImageList" -mf type=$FileType -ovc lavc -lavcopts vcodec=huffyuv:format=422p -oac copy -o huffyuv.avi -aspect 4:3 -force-avi-aspect 1.3333334
fi

if [[ $? != 0 ]]; then
	echo "mencoder did not finish properly! Exiting..."
	exit
fi

#either way, we need to do this
mplayer huffyuv.avi -vo yuv4mpeg

if [[ $? != 0 ]]; then
	echo "mplayer did not finish properly! Exiting..."
	exit
fi

mpeg2enc -f2 -b 10000 -n n -q 4 -H -o "$OutputFile".mpg stream.yuv

if [[ $? != 0 ]]; then
	echo "mpeg2enc did not finish properly! Exiting..."
	exit
fi

rm huffyuv.avi
rm stream.yuv


