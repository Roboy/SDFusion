.. _runtime_view:

Runtime View
============

.. todo::
  Add a runtime view of i.e. the startup of your code.
  This should help people understand how your code operates and where to look if something is not working.

  **Contents.**
  alternative terms:
  -  Dynamic view
  -  Process view
  -  Workflow view
  This view describes the behavior and interaction of the system’s
  building blocks as runtime elements (processes, tasks, activities,
  threads, …).

  Select interesting runtime scenarios such as:

  -  How are the most important use cases executed by the architectural
     building blocks?

  -  Which instances of architectural building blocks are created at
     runtime and how are they started, controlled, and stopped.

  -  How do the system’s components co-operate with external and
     pre-existing components?

  -  How is the system started (covering e.g. required start scripts,
     dependencies on external systems, databases, communications systems,
     etc.)?

      **Note**

      The main criterion for the choice of possible scenarios (sequences,
      workflows) is their **architectural relevancy**. It is **not**
      important to describe a large number of scenarios. You should rather
      document a representative selection.

    Candidates are:

    1. The top 3 – 5 use cases

    2. System startup

    3. The system’s behavior on its most important external interfaces

    4. The system’s behavior in the most important error situations

    **Motivation.**

    Especially for object-oriented architectures it is not sufficient to
    specify the building blocks with their interfaces, but also how
    instances of building blocks interact during runtime.

    **Form.**

    Document the chosen scenarios using UML sequence, activity or
    communications diagrams. Enumerated lists are sometimes feasible.

    Using object diagrams you can depict snapshots of existing runtime
    objects as well as instantiated relationships. The UML allows to
    distinguish between active and passive objects.

Runtime Scenario 1
------------------

.. figure:: images/IS-O_sequence.*
  :alt: Demo Sequence diagram

  **UML-type sequence diagram** - Shows how components interact with each other during runtime.


Runtime Scenario 2
------------------
...
