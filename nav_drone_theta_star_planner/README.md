## Theta Star Planner ##

A 3D implementation of the TetaStar Planning algorithm.

### Testing ###
This is a plugin to the Nav Drone Planner Server, and cannot be run on its own.

### Status ###
The algorithm runs, and returns a good path.  HOWEVER, if the start and finish locations are on different altitudes, the algorithm takes long to execute (150 seconds for a 7 meter path on a Raspberry Pi 3).
A bit of tunning might be successfull here.
