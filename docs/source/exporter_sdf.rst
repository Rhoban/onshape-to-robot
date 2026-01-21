SDF
===

Introduction
-------------

The `SDF format <http://sdformat.org/>`_ is an extension of URDF extensively used in ROS.


* When using SDF, frames will be exported as ``<frame>`` items.
* The links and joints always using ``relative_to`` attribute to specify the parent frame, keeping the same coordinates system as in URDF.
* Additionally to the ``robot.sdf`` file, a ``model.config`` file will be produced, adding metadata useful for Gazebo.

``config.json`` entries (SDF)
-----------------------------

Here is an example of complete ``config.json`` file, with details below:

.. code-block:: javascript

    {
        "url": "document-url",
        "output_format": "sdf",
        // ...
        // General import options (see config.json documentation)
        // ...

        // Additional XML file to be included in the URDF (default: "")
        "additional_xml": "my_custom_file.xml",

        // Override joint properties (default: {})
        "joint_properties": {
            // Default properties for all joints
            "*": {
                "max_effort": 10.0,
                "max_velocity": 6.0,
                "friction": 0.5
            },
            // Set the properties for a specific joint
            "joint_name": {
                "max_effort": 20.0,
                "max_velocity": 10.0,
                "friction": 0.1,
                "limits": [0.5, 1.2]
            },
            "wheel": {
                "type": "continuous"
            }
        },

        // Override geometry properties (default: {})
        "geom_properties": {
            // Set properties for specific links using pattern matching
            "tibia": {
                "collision": {
                    "mu": "1.2",
                    "mu2": "0.8"
                }
            },
            // Wildcard patterns are supported
            "leg_*": {
                "collision": {
                    "bounce": "0.5",
                    "max_contacts": "10"
                }
            }
        },
    }

``joint_properties`` *(default: {})*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Allow to specify the properties of the joints produced in the URDF output. The key should be joint names. The special ``default`` key will set default values for each joints.

Possible values are:

* ``max_effort``: The maximum effort that can be applied to the joint (added in the ``<joint effort=...>`` tag)
* ``max_velocity``: The maximum velocity of the joint (added in the ``<joint velocity=...>`` tag)
* ``friction``: The friction of the joint (added in the ``<joint_properties friction=...>`` tag)
* ``type``: Sets the joint type (changing the ``<joint type="...">`` tag)
* ``limits``: Override the joint limits, should be a list of two values (min, max)

``geom_properties`` *(default: {})*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Allow to specify the properties of the geometries (collision and visual) produced in the SDF output. The key should be part names with support for wildcard pattern matching.

Properties can be specified separately for ``visual`` and ``collision`` geometries, or applied to both if not nested.

Wildcard patterns (``*``, ``?``, ``[seq]``) are supported for matching part names. When multiple patterns match, properties are merged in order with later matches overriding earlier ones.

All properties are added as nested XML elements within the ``<visual>`` or ``<collision>`` tags. Common SDF geometry properties include:

* ``mu``, ``mu2``: Friction coefficients for collision geometries
* ``bounce``: Restitution coefficient for collision geometries
* ``max_contacts``: Maximum number of contact points for collision geometries
* ``kp``, ``kd``: Contact stiffness and damping parameters

``additional_xml`` *(default: "")*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to include additional XML in the URDF, you can specify the path to the file here. This file will be included in the produced SDF file.

.. note::

    Alternatively, ``additional_xml`` can be a list of files

