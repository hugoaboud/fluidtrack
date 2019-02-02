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

  1. Open Blender simulation at _sim/@/@.blend_
  
  2. Bake fluid simulation:
  ![bake](https://user-images.githubusercontent.com/2349393/52071516-56879d00-256a-11e9-9812-ae7c13876c29.gif)
  
  3. Run the _parse_cache_ script (more info below) to generate the .flow file
  > python scripts/parse_cache.py sim/brumadinho/cache_fluid sim/brumadinho/fluid.flow 50 [0, 0, 0.203386] [19, 18.5, 3]
  
  4. Run fluidtrack passing the desired config json as argument
  > ./fluidtrack sim/brumadinho/config.json
  
  5. To visualize the results, select the Object on Blender and run Blender script _import_anim_ to create position/orientation keyframes:
  ![import_anim](https://s2.gifyu.com/images/import_anim.gif)
  
#### Scripts

##### scripts/parse_cache.py

Parse the Blender fluid simulation info from the "sim/@/cache_fluid" folder into a single .flow file.
Arguments:
  1. cache folder
  2. output file
  3. animation length (seconds, from Blender fluid simulation Time)
  4. position array (from Blender Object transform)
  5. scale array (from Blender Object transform)
Example:
> python scripts/parse_cache.py sim/brumadinho/cache_fluid sim/brumadinho/fluid.flow 50 [0, 0, 0.35] [15, 12.5, 1]

##### scripts/export_terrain.py
Blender script for exporting the selected mesh as a .csv file containing the vertex position.

##### scripts/import_anim.py
Blender script for importing a .csv file containing coordinates to animate the position and rotation of the selected object.

![brumadinho](https://s2.gifyu.com/images/brumadinho2.gif)
