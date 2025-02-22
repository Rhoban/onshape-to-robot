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
  * Else, a ``connect`` constraint will be added.

* Additionally to the ``robot.xml`` file, a ``scene.xml`` file will be produced, adding floor and lighting useful for testing purpose.

``config.json`` entries (MuJoCo)
--------------------------------

Here is an example of complete ``config.json`` file, with details below:

.. code-block:: javascript

    {
        "document_id": "document-id",
        "output_format": "urdf",
        // ...
        // General import options (see config.json documentation)
        // ...

        // Override joint properties (default: {})
        "joint_properties": {
            // Default properties for all joints
            "default": {
                "actuated": true,
                "max_effort": 10.0,
                "friction": 0.5,
                "limits": [0.5, 1.2]
                // ...
            },
            // Set the properties for a specific joint
            "joint_name": {
                "max_effort": 20.0,
                "friction": 0.1
                // ...
            }
        },

        // Whether the collisions should be used in visual as well (default: false)
        "draw_collisions": true,
        // Use only pure shapes (when available) for collisions
        "collisions_no_mesh": true,
        // Whether to add a free joint to the root link (default: true)
        "freejoint": false,
        // Additional XML file to be included in the URDF (default: "")
        "additional_xml": "my_custom_file.xml",
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

    * ``kp``, ``kd`` and ``ki`` gains
    * ``forcerange``

``draw_collisions`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If this flag is set to ``true``, the same items that are used for collisions will be used for visuals as well. If you have pure shape approximations, this is useful for debugging purposes.

``collisions_no_mesh`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If this flag is set to ``true``, only pure shapes will be used for collisions, and not the mesh. This is useful for performance reasons.

``freejoint`` *(default: true)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If this flag is set to ``false``, a free joint will not be added to the root link. This will result in a fixed robot.

``additional_xml`` *(default: "")*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to include additional XML in the URDF, you can specify the path to the file here. This file will be included at the end of the URDF file.

