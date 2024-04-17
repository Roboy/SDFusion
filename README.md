# SDFusion
SDFusion is an add-on for **Fusion 360** that can automatically generate an SDF description of the model as well as export meshes or additional markers. Examples can be found [here](https://github.com/CARDSflow/robots/tree/master).

Table of contents
=================

<!--ts-->
   * [Installation](#installation)
   * [Usage](#usage)
      * [Preparing the robot in Fusion360](#preparing-the-robot-in-fusion360)
      * [Links](#links)
      * [Joints](#joints)
      * [Tendons](#tendons)
      * [Configs](#configs)
<!--te-->


Installation
============
1. Clone or download the SDFusion repository
  `git clone https://github.com/Roboy/SDFusion.git`
2. Open `Scripts and Add-Ins` dialog in Fusion 360 by pressing `Shift+S`
3. Click on the `Add-Ins` tab and press the green plus icon on the top
4. Navigate to the SDFusion folder and select it.
You should be able to run/debug the add-in from the `Scripts and Add-Ins` dialog now:

![Fusion 360 script dialog](https://github.com/Roboy/SDFusion/blob/master/images/scriptsdialog.png "Fusion 360 script dialog")

Usage
=====

Preparing the robot in Fusion360
-----
1. Switch Fusion360 design configuration to Z-axis up (default is Y up).
2. Break all links in the design at all hierarchy levels.
3. Create rigid groups for all links of the robot and call them `EXPORT_link_name`. For example, select all components that belong to `base` link, create a rigid group called `EXPORT_base`.
4. Define all joints between the links and call them `EXPORT_joint_name`. Select child as Component1 and parent as Component2. Child and parent component must belong to two different rigid groups, e.g. links. Supported joint types are: revolute, fixed and ball.
5. Define joint limits for the joints. Otherwise, min and max joint limit will be exported as 0.

Now the model is ready to be exported.

Links
-----
SDFusion requires all the links to be defined as rigid groups in Fusion 360. The script will automatically:
* create and export an STL mesh of the link
* calculate intertia and center of mass
* generate a `model.sdf` file with all rigid groups as links

Rigid group names have to follow the naming convetion: `EXPORT_link_name`. The rigid group has to contain all components (including children) that consitute the link. 

Joints
------
In order to export a joint, it has to be 
* defined between components that belong to different rigid groups
* called `EXPORT_joint_name`

*When creating a joint in Fusion 360, make sure you select the child first and then the parent.*
Currently, fixed, prismatic, revolute joint types are supported. A ball-and-socket joint can be emulated as a gimbal joint - a series of three 1-DoF revolute joints nested in each other.

Tendons
-------
SDFusion allows to export cable-driven robot models with multiple attachment points (via-points). 
The naming scheme defined for the viapoints is the following: `VP_motornumber_EXPORT_linkname_viapointnumber` (e.g. `VP_motor0_EXPORT_base_0`). You can either add them manually as construction points in Fusion 360 or use `ViaPoints` tab from the plugin.

![SDFusion viapoints tab](https://github.com/Roboy/SDFusion/blob/master/images/viapointstab.png "Viapoints tab")

Here, you have to specify the motor number and the link name. Press `Select` and the viapont number will increment automatically as you click on *circular edges* in your design. The newly created attachement points are listed under the `Construction` in the Fusion 360 browser. 

![SDFusion viapoints](https://github.com/Roboy/SDFusion/blob/master/images/viapoints.png "Attachment points")

Refer to [CARDSflow](https://github.com/CARDSflow/CARDSflow) to learn about possible options to control your cable robot. 

Configs
-------

![SDFusion configs](https://github.com/Roboy/SDFusion/blob/master/images/configs.png "SDFusion configs")

Configuration | Explanation
--- | ---
updateRigidGroups | by default the script will *cache* your rigid groups, as each of them is copied to a new component. This is done to save time for the subsequent exports, since this step is time consuming. If you add or remove components from a rigid group, check this box to update it.
exportMeshes | creates a folder with `stl` meshes for each rigid group
sdf | `model.sdf` is created
viapoints |`model.sdf` will include descriptions of viapoints
caspr | only valid when viapoints are defined and checked; generates [CASPR](https://github.com/darwinlau/CASPR) files with definitions of bodies and cables, that are required for controlling the robot
opensim | generates a separate `muscles.osim` file with muscle descriptions in OpenSim format
darkroom | exports visual markers defined on the robot
remove parts smaller 1g | deletes all components in the design with mass less than 1g
self_collide | `<self_collide>false</self_collide>` tag in `model.sdf`
dummy_inertia | ignores real inertia values calculated from your design
