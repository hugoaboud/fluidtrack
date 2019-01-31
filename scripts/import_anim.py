##
#   FluidMap
#   Open Source library for tracking objects on terrain fluid flows
#
#   [import_anim.py]
#    Imports a .csv containing object positions to keyframes for the
#   selected object in Blender
#
#   @author: Hugo Aboud
##

import bpy

file = open('/home/aboud/fluidtrack/sandbox.csv')
frames = file.readlines()

obj = bpy.context.object
obj.animation_data_clear()
for f in range(len(frames)):
    pos = frames[f].split(",")
    obj.location = [float(pos[0]), float(pos[1]), float(pos[2])]
    obj.rotation_quaternion = [float(pos[3]), float(pos[4]), float(pos[5]), float(pos[6])]
    obj.keyframe_insert(data_path="location", frame=f)
    obj.keyframe_insert(data_path="rotation_quaternion", frame=f)
