#!/bin/bash

# parse input argument
if [ "$1" != "" ]; then
    path="$1"
else
    # default path
    d=`date +%Y%m%d`    # get date as YYYYMMDD
    path=identification_data/$d
fi

# create folder if not existent
if [ ! -d $path ]; then
    mkdir -p $path;
fi

# convert bagfiles
for bagfile in bagfiles/*.bag; do
    echo converting `basename "$bagfile"` to csv ...;
    n=${bagfile%.*}          # remove the extension `.bag`
    n=${n#"${n%_*}_"}     # remove up to the last underscore `_`
    for topic in eta nu controls p_desired; do
        rostopic echo -p -b ${bagfile} $topic > ${path}/${topic}_$n.csv ; 
    done
done