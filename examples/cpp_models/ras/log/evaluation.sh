#!/usr/bin/sh


densities=(
    0.001
    0.003
    0.005
    0.007
    0.009
    0.01
    0.013
    0.015
    0.017
    0.019
    0.02
    0.025
    0.03
    0.035
    0.04
    0.045
    0.05
#    0.06
#    0.07
#    0.08
#    0.09
#    0.1
)

for density in "${densities[@]}"; do
    for i in {1..10}; do
        ../build/ras -p DESPOT -d ${density}
    done
done

policies=(
    "MYOPIC"
    "EGOISTIC"
    "REFERENCE"
)

while read log ; do
    for policy in "${policies[@]}"; do
        ../build/ras -p ${policy} -l ${log}
    done
done < <(find . -name "*.json" )
