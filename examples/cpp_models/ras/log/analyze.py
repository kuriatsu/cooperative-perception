import json
import sys

with open(sys.argv[1], "r") as f:
    data = json.load(f)

for frame in data:

