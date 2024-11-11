import re

# Expected ASCII data in the format <x,y>
format = re.compile(r"<(-?\d+\.\d+),\s*(-?\d+\.\d+)>")

def parse_coord(data):
    match = format.match(data)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        return x, y
    else:
        print("Data format error")
        return None