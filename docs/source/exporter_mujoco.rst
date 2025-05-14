.. _exporter-mujoco:

MuJoCo
======

Introduction
-------------

MuJoCo is a standard physics simulator, coming with an extensive description format.

* Frames will be added as ``site`` tags in the MuJoCo XML file.
* *Actuators* will be created for all actuated joints (see below).
* When :ref:`kinematic loops <kinematic-loops>` are present, they will be enforced using equality constraints.

  * If the loop is achieved using a ``fixed`` connector, a ``weld`` constraint will be added.
  * If a ``ball`` joint is used, a ``connect`` constraint will be added
  * If a ``revolute`` joint is used, two ``connect`` constraints will be used

* Additionally to the ``robot.xml`` file, a ``scene.xml`` file will be produced, adding floor and lighting useful for testing purpose.

``config.json`` entries (MuJoCo)
--------------------------------

Here is an example of complete ``config.json`` file, with details below:

.. code-block:: javascript

    {
        "url": "document-url",
        "output_format": "mujoco",
        // ...
        // General import options (see config.json documentation)
        // ...

        // Additional XML file to be included in the URDF (default: "")
        "additional_xml": "my_custom_file.xml",

        // Override joint properties (default: {})
        "joint_properties": {
            // Default properties for all joints
            "*": {
                "actuated": true,
                "forcerange": 10.0,
                "frictionloss": 0.5,
                "limits": [0.5, 1.2]
                // ...
            },
            // Set the properties for a specific joint
            "joint_name": {
                "forcerange": 20.0,
                "frictionloss": 0.1
                // ...
            }
        },

        // Override equality attributes
        "equalities": {
            "closing_branch*": {
                "solref": "0.002 1",
                "solimp": "0.99 0.999 0.0005 0.5 2"
            }
        }
    }

``joint_properties`` *(default: {})*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Allow to specify the properties of the joints produced in the URDF output. The key should be joint names. The special ``default`` key will set default values for each joints.

Possible values are:

* ``actuated``: *(default: true)* whether an actuator should be associated to this joint,
* ``class``: a ``class="..."`` to be added to the joint (and actuator) 
* ``type`` *(default: position)* defines the actuator that will be produced
* ``range`` *(default: true)*: if ``true``, the joint limits are reflected on the joint ``range`` attribute
* ``limits``: Override the joint limits, should be a list of two values (min, max)

* The following are reflected as ``<joint ...>`` attributes:

    * ``frictionloss``
    * ``damping``
    * ``armature``
    * ``stiffness``

* The following are reflected as actuator (``<position ...>`` or other) attributes:

    * ``kp``, ``kv`` and ``dampratio`` gains
    * ``forcerange``

``equalities`` *(default: {})*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This entry allows to override the equality attributes of the MuJoCo XML file. The key should be the equality name (which might contains wildcards ``*``), and the value should be a dictionary of attributes.

This can be used to adjust the ``solref`` and ``solimp`` attributes of the equality constraints.

``additional_xml`` *(default: "")*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to include additional XML in the URDF, you can specify the path to the file here. This file will be included in the produced XML.

.. note::

    Alternatively, ``additional_xml`` can be a list of files