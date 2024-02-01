#stolen from Aron Harder @ UVA

import argparse
import re

# TODO: This breaks with UdpMessage.proto, where the count should NOT reset after the oneof{}
# TODO: This breaks if there are multiple '{}' on a line, or a field on a line with a '{}'

parser = argparse.ArgumentParser(description='Renumber the fields in a proto file.')
parser.add_argument("-f","--file", action='store',type=str, help="The location of the proto file")

# Read parameters from command line
args = parser.parse_args()
fileloc = args.file

pattern = re.compile('(.*=\s*)\d+(\s*[^\d\s])') # Identify lines that are fields and avoid lines like 'syntax = "proto3";'

with open(fileloc,"r") as _in:
    lines = _in.readlines()

output = []
ids_ = [] # Field numbers, can be recursive
for line in lines:
    if '{' in line: # Start of new block, add new field number counter
        if 'enum' in line: # Enums start at 0
            ids_.append(0)
        else: # Fields start at 1
            ids_.append(1)
        output.append(line.rstrip()) # No changes needed, add line to final output
    elif '}' in line: # End of block, return to previous field number counter
        ids_.pop()
        output.append(line.rstrip()) # No changes needed, add line to final output
    elif pattern.search(line) and len(ids_) > 0:
        temp = pattern.split(line) # Remove the number from the line while preserving all other data
        temp.insert(2,str(ids_[-1])) # Insert correct field number
        output.append(''.join(temp).rstrip()) # Add line to final output
        ids_[-1]+=1 # Increment field number
    else:
        output.append(line.rstrip()) # No changes needed, add line to final output

with open("output.proto","w") as _out:
    _out.write("\n".join(output))
