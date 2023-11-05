#!/bin/zsh

files=()

while read file ; do
    files+=${file}
done < <(find . -name "*.json" )

/home/kuriatsu/Source/analize_env/bin/python3 ../log/analyze.py ${files}
# /home/kuriatsu/Source/venv_analyze/bin/python3 ../log/analyze.py ${files}
# python3 analyze.py ${files}
