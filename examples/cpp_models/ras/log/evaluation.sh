#!/usr/bin/sh

policies = (
    "POMDP"
    "MYOPIC"
    "EGOISTIC"
)

dencities = (
    0.001
    0.005
    0.01
    0.02
    0.03
    0.04
    0.05
    0.06
    0.07
    0.08
    0.09
    0.1
)

for policy in "${policies[@]}"; do
    for dencity in "${dencities[@]}"; do
        for i in {1..10}; do
            ../build/ras ${policy} ${dencity}
        done
    done
done
