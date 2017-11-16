.. _building_block_view:

Building Block View
===================

.. _overview:

Overview
--------

.. todo::
  Inset a building block view:

  - Static decomposition of the system into building blocks and the relationships thereof.
  - Description of libraries and software used
  -

  We specify the system based on the blackbox view from :ref:`context_within_environment` by now considering it a whitebox and identifying the next layer of blackboxes inside it. We re-iterate this zoom-in until specific granularity is reached - 2 levels should be enough.

  **Motivation.**

  This is the most important view, that must be part of each architecture
  documentation. In building construction this would be the floor plan.

  **Tool**

  - Create diagrams as below.

.. _bb-l1-overview:

The white box view of the first level of your code.
This is a white box view of your system as shown within the in Context in figure: :ref:`context_within_environment`.
External libraries and software are clearly marked.

.. _building-block-overview:

.. figure:: images/05_building_blocks.png
   :alt: Bulding blocks overview

   Building blocks overview


.. _bb_l1_component_list:

Level 1 - Components
--------------------
The highest level components

.. todo::
    - Define your groups using the **/defgroup** doxygen command
    - Add **@addtogroup** tags to doxygen blocks of components in code as described here: http://www.stack.nl/~dimitri/doxygen/manual/grouping.html#modules
    - Adapt the doxygencall to match the group name

.. _bb-l1-components:
 doxygengroup:: nuttygroup

.. _bb_l2_component_list:

Level 2 - Components within each component of level 1
-----------------------------------------------------

.. todo::
    - Define your groups using the **/defgroup** doxygen command
    - Add **addtogroup** tags to doxygen blocks of components in code as described here: http://www.stack.nl/~dimitri/doxygen/manual/grouping.html#modules
    - Adapt the doxygencall to match the group name

.. _bb-l2-components:
 doxygengroup:: nuttygroup2
