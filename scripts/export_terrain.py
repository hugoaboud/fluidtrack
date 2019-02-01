##
#   FluidMap
#   Open Source library for tracking objects on terrain fluid flows
#
#   [export_terrain.py]
#    Blender scripts that exports the vertex of selected mesh as a .csv file
#
#   @author: TLousky
##

import bpy

outputFile = 'set/some/file/here'

verts = [ bpy.context.object.matrix_world * v.co for v in bpy.context.object.data.vertices ]

csvLines = [ ",".join([ str(v) for v in co ]) + "\n" for co in verts ]

f = open( outputFile, 'w' )
f.writelines( csvLines )
f.close()
