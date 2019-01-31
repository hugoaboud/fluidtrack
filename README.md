# fluidtrack
Open Source library for tracking objects on terrain fluid flows

#### Dependencies

- Python
- ReactPhysics3D C++ Library
- Blender (for running the fluid simulation and visualizing the results)

#### Setup

>git clone https://github.com/hugoaboud/fluidtrack  
cd fluidtrack  
make

#### Usage

##### scripts/export_terrain.py
Blender script for exporting the selected mesh as a .csv file containing the vertex position.

##### scripts/import_anim.py
Blender script for importing a .csv file containing coordinates to animate the position and rotation of the selected object.

##### scripts/parse_cache.py

Use this script to parse the Blender fluid simulation info from the "sim/@/cache_fluid" folder into a single .flow file. You must also specify the animation time in integer seconds.
> python scripts/parse_cache.py sim/sandbox/cache_fluid sim/sandbox/sandbox.flow 2

##### fluidtrack

The executable takes 3 arguments:
  - terrain: .csv file located on the simulation folder (generated with export_terrain)
  - flow: .flow file located on the simulation folder (generated with parse_cache)
  - output: name of the output file

The output is a .csv containing the object position (3 values) and orientation (4 values - quaternion).

> ./fluidtrack sim/sandbox/sandbox.csv sim/sandbox/sandbox.flow sandbox.csv

![sandbox](https://s2.gifyu.com/images/sim3.gif)
