URDF
====

Introduction
-------------

URDF is a very standard format that can be exported by ``onshape-to-robot``. Below are the specific configuration entries that can be specified when using this format.

When using this exporter, frames will be added as a *dummy links* attached to their body using a fixed joint

``config.json`` entries (URDF)
------------------------------

Here is an example of complete ``config.json`` file, with details below:

.. code-block:: javascript

    {
        "url": "document-url",
        "output_format": "urdf",
        // ...
        // General import options (see config.json documentation)
        // ...

        // Package name (for ROS) (default: "")
        "package_name": "my_robot",
        // Additional XML file to be included in the URDF (default: "")
        "additional_xml": "my_custom_file.xml",
        // Exclude inertial data for fixed bodies (default: false)
        "set_zero_mass_to_fixed": true,

        // Override joint properties (default: {})
        "joint_properties": {
            // Default properties for all joints
            "default": {
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

``package_name`` *(default: "")*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you are exporting a URDF for ROS, you can specify the package name here. This will be used in the ``<robot>`` tag.

``additional_xml`` *(default: "")*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to include additional XML in the URDF, you can specify the path to the file here. This file will be included in the produced URDF file.

.. note::

    Alternatively, ``additional_xml`` can be a list of files

``set_zero_mass_to_fixed`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This option sets the mass to 0 for bodies that are :ref:`fixed <fixed-robot>` to the world.

.. note::

    In PyBullet, such bodies are indeed recognized as fixed