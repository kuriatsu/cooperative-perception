#!/usr/bin/sh


densities=(
    0.001
    0.002
    0.003
    0.004
    0.005
    0.006
    0.007
    0.008
    0.009
    0.01
    0.013
    0.015
    0.017
    0.019
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

param_file="../param1.json"

for density in "${densities[@]}"; do
    for i in {1..10}; do
        ../build/ras --policy DESPOT --density ${density} --param ${param_file} 
    done
done

policies=(
    "MYOPIC"
    "EGOISTIC"
    "REFERENCE"
)

while read log ; do
    for policy in "${policies[@]}"; do
        buf=${log##*[A-Z]}
        ../build/ras --policy ${policy} --log ${log} --density ${buf%_*} --param ${param_file} 
    done
done < <(find . -name "*.json" )
