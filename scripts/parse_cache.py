##
#   FluidMap
#   Open Source library for tracking objects on terrain fluid flows
#
#   [parse_cache.py]
#    Script that reads a Blender "cache_fluid" folder and outputs
#   a .flow binary file containing vectors for each simulation frame
#
#   usage:
#   python parse_cache.py <cache_folder> <output_file> <anim_time>
#
#   @author: Hugo Aboud
##

import numpy as np
from subprocess import check_call
import os
import struct
import sys

if (len(sys.argv) < 2):
    exit("Error: No cache folder specified")
else:
    cache_folder = sys.argv[1]

if (len(sys.argv) < 3):
    exit("Error: No output file specified")
else:
    output_path = sys.argv[2]

if (len(sys.argv) < 4):
    exit("Error: No animation time specified")
else:
    anim_time = int(sys.argv[3])

prefix = "fluidsurface_final_"

location = [0, 0, 1.5]
scale = [3, 3, 3]

# Count how many frames are in cache
count = 0
for file in os.listdir(cache_folder):
    sfile = file.split(prefix)
    if (len(sfile) == 2):
        index = int(sfile[1].split(".")[0])
        if index > count: count = index

print("Frames: "+str(count));

# Create output file
output_file = open(output_path, 'wb');
output_file.write(count.to_bytes(4, 'little'));
output_file.write(anim_time.to_bytes(4, 'little'));

# Function to Parse frame by index
for index in range(count):
    strindex = "{0:04d}".format(index)
    print("Parsing "+strindex);
    check_call(['gunzip', cache_folder+prefix+strindex+".bobj.gz"])
    check_call(['gunzip', cache_folder+prefix+strindex+".bvel.gz"])
    fobj = open(cache_folder+prefix+strindex+".bobj", "r")
    fvel = open(cache_folder+prefix+strindex+".bvel", "r")
    vcount = np.fromfile(fobj, np.uint32, 1)[0]
    vcount = np.fromfile(fvel, np.uint32, 1)[0]
    print("\tvertex count: " + str(vcount))
    output_file.write(int(vcount).to_bytes(4, 'little'))
    for _ in range(vcount):
        vertex = np.fromfile(fobj, np.float32, 3)
        vel = np.fromfile(fvel, np.float32, 3)
        np.array([
            vertex[0]*scale[0]+location[0],
            vertex[1]*scale[1]+location[1],
            vertex[2]*scale[2]+location[2],
            vel[0]*scale[0],
            vel[1]*scale[1],
            vel[2]*scale[2]
        ]).astype(np.float32).tofile(output_file)
    check_call(['gzip', cache_folder+prefix+strindex+".bobj"])
    check_call(['gzip', cache_folder+prefix+strindex+".bvel"])

output_file.close()
