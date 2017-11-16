.. _solution_strategy:

Solution Strategy
=================

General Strategy
----------------

Using the Autodesk Fusion API you have access to nearly all information stored for a robot model.

The script uses a common XML parser to generate the SDF file. For more information about SDF in general see http://sdformat.org/spec and for additional information **see below**.

The mesh files for links of the robot are exported by finding all rigid groups of the model. As you can only export single components to the STL mesh format, all parts of the rigid group are copied to a new component. The link information is then parsed into the SDF file.

The joints of the robot are exported by finding all components of the model and finding their joints, as every joint is just associated to one component. The Fusion API provides a function that should retrieve all joints of a robot model, but it is not used here as it is implemented erroneous. After that the script finds the associated rigid groups to these joints. At this point there is also checked for non-exported links. These occur when a single part of the robot model resembles a link on its own, because then it can't be defined as a rigid group.

The script finishes with a simple User Dialog in Fusion.

Additional Information about SDFormat and Gazebo
------------------------------------------------

Fusion uses a right-handed coordinate system with the y-axis point up. Gazebo uses a right-handed coordinate system with the z-Axis pointing up. To handle this, the links of the robot model are rotated around the x-axis. This can be seen in the SDF file in the poses of the links. Everything else, like COM of links or associated joints, doesn't have to be rotated again!

The poses in the SDF file are given as position (x-y-z) and rotation (roll-pitch-yaw).

Fusion does not always use SI units, so sometimes the values of physical parameters are recalculated to be in SI units.

Joints: Upper (and lower) joint limits in the SDF file (Gazebo) are the negated maximum (and  minimum) joint limits retrieved from Fusion. This is due to the fact, that Gazebo rotates the child link of a joint around the parent link, and Fusion rotates the parent link around the child link.