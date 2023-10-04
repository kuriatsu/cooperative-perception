#!/bin/zsh

files=()

while read file ; do
    files+=${file}
done < <(find . -name "*.json" )

/home/kuriatsu/Source/venv_analyze/bin/python3 analyze.py ${files}
