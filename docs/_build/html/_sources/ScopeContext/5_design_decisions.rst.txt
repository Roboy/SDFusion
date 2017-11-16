.. _design_decisions:

Design Decisions
================

The script shall not modify the given Fusion Robot model at all.

Export to STL
-------------

Fusion can only export single components to STL (so not numerous components to one STL file). The aim was to only have one STL file per robot link, therefore all components of this link needed to be concluded in one single component. There are two ways to archieve this. First, generate a new component and move all subcomponents of the link to this new component. Before the script finishes all these actions need to be reversed. Second, generate a new component, and copy all subcomponents of the link to the new component. Before the script finishes the new component simply needs to be deleted. 

The second option was implemented. Copying all components takes a lot of time, but the script does not need to be executed very often. Deleting a new component is easier than keeping track of how many actions need to be reversed to reach the goal of not modifying the robot model.