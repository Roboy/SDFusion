.. _getting_started:

Getting started
===============

Make sure that your robot model is specified correctly (**see below**) before running the script. Furthermore you have to adapt the script to your use case (**see below**).

To run the script in Fusion, run the Script and Add-Ins command and select the SDFusion script. Click the "Run" button. The script automatically exports robot links and joints (and joint limits) to SDFormat and creates the required mesh files. For large robot models the script takes several minutes to finish and Fusion will freeze during that time. When the scipt is finished a success dialog is shown in Fusion. You now find your files in the desired location.

.. _model_specification:

Model specification
-------------------

A good practice is to generate a dedicated simulation copy of the Fusion robot model.

Therein all links of the robot have to be designed as rigid groups. All rigid groups will be exported as mesh (STL) files and will be listed in the model SDF file. make sure, taht all rigid groups have a unique name!

At the moment the following joint types are supported: fixed, cylindrical and ball joint. For cylindrical joints the joint limits are automatically exported. For ball joints the joint limits can't be exported, as this is not supported by the SDFormat. Make sure that all joints have a unique name!

Make sure to select your robot model as the **active product**, otherwise your SDF file will be empty.

.. _adapting_the_script:

Adapting the script
-------------------

Two lines of the script need to be modified by the user:

The variable **fileDir** specifies the location where the generated model.sdf file will be written to. Make sure that this folder contains a folder called "meshes".

The variable **modelName** specifies the name of the robot model. This is the name under which you will find the model in Gazebo.

.. figure:: images/GlobalVariables.png
  :alt: Change these variables