
Commands
========

Here are the commands provided by `onshape-to-robot`

``onshape-to-robot`` - import a robot
-------------------------------------

This is the main script that run the robot import.

* Check out design time considerations, and how to create :doc:`your own config.json <config>` file
* See the `examples of robots <https://github.com/rhoban/onshape-to-robot-examples>`_ if you want to try it

Usage:

.. code-block:: bash

    onshape-to-robot [directory containing config.json]


``onshape-to-robot-edit-shape`` - editing pure shape
----------------------------------------------------

This generates a ``.scad`` file and opens it, allowing you to editing the pure shape approximation of some mesh.

* Check out the :doc:`pure shape <pure-shapes>` page for more information

Usage:

.. code-block:: bash

    onshape-to-robot-edit-shape [stl file]

``onshape-to-robot-bullet`` - running bullet simulation to test your robot (URDF)
---------------------------------------------------------------------------------

This script starts a bullet simulation with the given robot that was imported with onshape-to-robot.

Usage:

.. code-block:: bash

    onshape-to-robot-bullet [-f] [directory containing robot.urdf]

The optional ``-f`` flag can be passed to fix th robot base (useful for robotics arms for example).

Note also that:
* If a joint name ends with ``_speed``, it will control its with speed instead of position.
* If a joint name ends with ``_passive``, it will not be controlled.
* If a joint specify its limits, they will be used for side sliders.

``onshape-to-robot-clear-cache`` - clearing cache
-------------------------------------------------

``onshape-to-robot`` uses cache for further call to avoid re-fetching all the data each time. You can use this
command to clear the cache.

Usage::

.. code-block:: bash

    onshape-to-robot-clear-cache
