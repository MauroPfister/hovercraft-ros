for bagfile in bagfiles/*.bag; do
    echo converting `basename "$bagfile"` to csv ...;
    n=${bagfile%.*}          # remove the extension `.csv`
    n=${n#"${n%_*}_"}     # remove up to the last underscore `_`
    for topic in pose controls; do
        rostopic echo -p -b ${bagfile} $topic > data_id/${topic}_$n.csv ; 
    done
done