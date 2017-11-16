.. _context:

Context
=======

System - Script - Fusion

Script uses System to store SDF files and STL mesh files.
Script uses Fusion API to retrieve all information about the robot model from Fusion
Script sends data to Fusion (creating new components, but deletes the data again)
Script does not modify the robot model - everything that is done is reversed - all new created is deleted afterwards

.. todo::
  Create a black box view of your software within its intendend environment.
  Identify all neighboring systems and specify all logical business data that is
  exchanged with the system under development.


.. _context_within_environment:
.. figure:: images/uml_system_context.*
  :alt: Bulding blocks overview

  UML System Context

  **UML-type context diagram** - shows the birds eye view of the system (black box) described by this architecture within the ecosystem it is to be placed in. Shows orbit level interfaces on the user interaction and component scope.
