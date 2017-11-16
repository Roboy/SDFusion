.. Software Documentation template master file, created by
   sphinx-quickstart on Fri Jul 29 19:44:29 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _welcome:

Welcome to SDFusion!
===========================================================

This project provides an exporter for Autodesk Fusion 360 robot models to SDFormat. This SDF file can then be used to simulate the robot model ín Gazebo, an open source robot simulator. Simulation is an important aspect in robotics, as it enables multiple people to work with one robot at the same time, as well as giving researchers the freedom to test new controls without damaging the robot.

The exporter itself is a python script, which needs to be loaded into Autdesk Fusion 360 and executed.

.. _background_prerequisits:

Relevant Background Information and Pre-Requisits
--------------------------------------------------

**Use:**

To use this script you need a machine running Windows with Autodesk Fusion 360 installed. Furthermore your robot model needs to be in Fusion and you need to modify it as specified here. Please make yourself familiar with the concept of joints that Autodesk Fusion uses, opposed to the concept of constraining motion that most other CAD programs use.

**Develop:**

To develop this script further you need to be familiar with python, the SDFormat and the Autodesk Fusion API. Furthermore you need a basic understanding of how CAD programs work internally. Additionally you need to know about robot links, joints, kinematic chains and rotation matrices. You can find the Fusion API User's and Reference Manual here: http://help.autodesk.com/view/fusion360/ENU/?guid=GUID-A92A4B10-3781-4925-94C6-47DA85A4F65A .

Helpful Links:

- Autodesk Fusion 360		http://www.autodesk.de/products/fusion-360/overview
- SDFormat 					http://sdformat.org/
- Gazebo 					http://gazebosim.org/

.. _requirements_overview:

Requirements Overview
---------------------

The **software requirements** define the system from a blackbox/interfaces perspective. They are split into the following sections:

- **User Interfaces** - :ref:`user_interfaces`
- **Technical Interfaces** - :ref:`technical_interfaces`
- **Runtime Interfaces and Constraints** - :ref:`runtime_interfaces`

Contents:

.. _usage:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Usage and Installation

  Usage/*

.. _ScopeContext:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Interfaces and Scope

  ScopeContext/*

.. _development:
.. toctree::
  :maxdepth: 1
  :glob:
  :caption: Development

  development/*

.. toctree::
   :maxdepth: 1

   about-arc42
