#!/bin/bash 

STARTDIR=`pwd`
#Switch to this directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

THEDATETAG=`date +"%y-%m-%d_%H-%M-%S"`

avconv -i colorFrame_0_%05d.pnm -y -r 20 -threads 8 -b 30000k -s 640x480  outHD_$THEDATETAG.mp4 

cd $STARTDIR 
exit 0

