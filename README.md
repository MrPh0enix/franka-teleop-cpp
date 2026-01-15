# Modification of the teleop framework for the grasping experiment.


### Starting the Robots:

1. Make sure both the Franka robots and their PCs are switched on.
1. Open a web browser (preferably Chrome) and connect to both the Frankka Control Interfaces on: 172.22.2.3/desk and 172.22.2.4/desk.
2. Unlock the brakes on the arms and activate FCI.


### Compiling:

This step is only necessary if you are switching back from the "master" branch.

1. In the base folder, open a terminal and run:
2. '''rm -r build'''
3. '''mkdir build && cd build'''
4. '''cmake ..'''
5. '''make'''

### After compiling, or if you already have a compiled program:

1. In the build folder in two separate terminals, run:
   '''./leader <../lookups/lookup_file.csv>'''
   and
   '''./follower'''

