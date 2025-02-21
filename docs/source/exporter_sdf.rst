SDF
===

Introduction
-------------

When using SDF, frames will be exported as ``<frame>`` items.

The links and joints always using ``relative_to`` attribute to specify the parent frame, keeping the same coordinates system as in URDF.

``config.json`` entries (URDF)
-----------------------

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

        // Whether the collisions should be used in visual as well (default: false)
        "draw_collisions": true,
        // Use only pure shapes (when available) for collisions
        "collisions_no_mesh": true,
        // Additional XML file to be included in the URDF (default: "")
        "additional_xml": "my_custom_file.xml",
    }

``joint_properties`` *(default: {})*
~~~~~~~~~~~~~~

Allow to specify the properties of the joints produced in the URDF output. The key should be joint names. The special ``default`` key will set default values for each joints.

Possible values are:

* ``max_effort``: The maximum effort that can be applied to the joint (added in the ``<joint effort=...>`` tag)
* ``max_velocity``: The maximum velocity of the joint (added in the ``<joint velocity=...>`` tag)
* ``friction``: The friction of the joint (added in the ``<joint_properties friction=...>`` tag)
* ``type``: Sets the joint type (changing the ``<joint type="...">`` tag)
* ``limits``: Override the joint limits, should be a list of two values (min, max)

``draw_collisions`` *(default: false)*
~~~~~~~~~~~~~~~~~~

If this flag is set to ``true``, the same items that are used for collisions will be used for visuals as well. If you have pure shape approximations, this is useful for debugging purposes.

``collisions_no_mesh`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~

If this flag is set to ``true``, only pure shapes will be used for collisions, and not the mesh. This is useful for performance reasons.

``additional_xml`` *(default: "")*
~~~~~~~~~~~~~~~~

If you want to include additional XML in the URDF, you can specify the path to the file here. This file will be included at the end of the URDF file.

