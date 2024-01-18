#!/usr/bin/sh


densities=(
#    0.001
    0.002
#    0.003
    0.004
#    0.005
    0.006
#    0.007
    0.008
#    0.009
    0.01
    0.012
    0.014
    0.016
    0.018
    0.02
#    0.025
#    0.03
#    0.035
#    0.04
#    0.045
#    0.05
#    0.06
#    0.07
#    0.08
#    0.09
#    0.1
)

param_files=(
"../param/param0.json"
"../param/param1.json"
"../param/param2.json"
"../param/param3.json"
"../param/param4.json"
"../param/param5.json"
)

for density in "${densities[@]}"; do
    for param in "${param_files[@]}"; do
        for i in {1..10}; do
            ../build/ras --policy OURS --density ${density} --param ${param} 
        done
    done
done

policies=(
    "MYOPIC"
    "NOREQUEST"
    "MYOPIC_PLUS"
    "MYOPIC_CONSERVATIVE"
    "REFERENCE"
)

while read log ; do
    for policy in "${policies[@]}"; do
        buf=${log##*[A-Z]}
        ../build/ras --policy ${policy} --log ${log} --density ${buf%_*}
    done
done < <(find . -name "*.json" )

while read log ; do
    python3 analyze.py ${log} 
done < <(find . -name "*.json" )
