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

        // Override geometry properties (default: {})
        "geom_properties": {
            // Replace mesh collision with capsule geometry at body origin
            "foot": {
                "collision": {
                    "type": "capsule",
                    "size": "0.005",
                    "fromto": "0 0 -0.01 0 0 0.01",
                    "pos": null,
                    "quat": null,
                    "friction": "1.2 0.005 0.0001"
                }
            },
            // Multiple colliders - approximate complex shape with primitives
            "foot_2": {
                "collision": [
                    {
                        "type": "capsule",
                        "size": "0.005",
                        "fromto": "0 0 -0.01 0 0 0",
                        "pos": null,
                        "quat": null
                    },
                    {
                        "type": "sphere",
                        "size": "0.008",
                        "pos": "0 0 0.01",
                        "quat": null
                    }
                ]
            },
            // Wildcard patterns are supported
            "leg_*": {
                "collision": {
                    "solimp": "0.9 0.95 0.001",
                    "solref": "0.02 1"
                },
                "visual": {
                    "rgba": "1 0 0 1"
                }
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

``geom_properties`` *(default: {})*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Allow to specify the properties of the geometries (collision and visual) produced in the MuJoCo output. The key should be part names with support for wildcard pattern matching.

Properties can be specified separately for ``visual`` and ``collision`` geometries, or applied to both if not nested.

Wildcard patterns (``*``, ``?``, ``[seq]``) are supported for matching part names. When multiple patterns match, properties are merged in order with later matches overriding earlier ones.

**Multiple Colliders:** The ``collision`` property can be an array to define multiple collision geometries for a single part. This is useful for approximating complex mesh shapes with multiple primitives for better physics performance.

All properties are added as XML attributes to the ``<geom ...>`` tag. **Properties specified in the config take priority and will override default values**, including the geometry type.

Common MuJoCo geom attributes include:

* ``type``: Override the geometry type (e.g., ``"capsule"``, ``"box"``, ``"sphere"``, ``"cylinder"``)
* ``size``: Size parameters for the geometry type (e.g., ``"0.005"`` for capsule radius when using ``fromto``)
* ``name``: Override the geometry name
* ``friction``: Friction coefficients (e.g., ``"1.2 0.005 0.0001"``)
* ``solimp``: Solver impedance parameters (e.g., ``"0.9 0.95 0.001"``)
* ``solref``: Solver reference parameters (e.g., ``"0.02 1"``)
* ``contype``: Contact type bitmask (e.g., ``"1"``)
* ``conaffinity``: Contact affinity bitmask (e.g., ``"1"``)
* ``rgba``: Color and transparency (e.g., ``"1 0 0 1"``)

.. note::

    You can replace mesh geometries with primitive shapes (capsule, box, sphere, cylinder) by setting the ``type`` and ``size`` attributes. When the type is overridden, mesh-specific attributes are automatically excluded.

.. note::

    To remove default attributes (such as ``pos`` or ``quat``), set them to ``null`` in the JSON config. For example:

    .. code-block:: javascript

        "geom_properties": {
            "foot": {
                "collision": {
                    "type": "capsule",
                    "size": "0.005",
                    "fromto": "0 0 -0.01 0 0 0.01",
                    "pos": null,
                    "quat": null
                }
            }
        }

    This is useful when replacing mesh geometries with primitives and you want the geometry at the body origin.

.. note::

    **Multiple Colliders** - You can approximate complex shapes with multiple primitives by providing an array:

    .. code-block:: javascript

        "geom_properties": {
            "hand": {
                "collision": [
                    {
                        "type": "box",
                        "size": "0.02 0.04 0.01",
                        "pos": "0 0 0"
                    },
                    {
                        "type": "sphere",
                        "size": "0.015",
                        "pos": "0 0.02 0"
                    },
                    {
                        "type": "sphere",
                        "size": "0.015",
                        "pos": "0 -0.02 0"
                    }
                ]
            }
        }

    Each array entry generates a separate ``<geom>`` tag, allowing you to build complex collision shapes from primitives. This often provides better simulation performance than mesh collisions.

``equalities`` *(default: {})*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This entry allows to override the equality attributes of the MuJoCo XML file. The key should be the equality name (which might contains wildcards ``*``), and the value should be a dictionary of attributes.

This can be used to adjust the ``solref`` and ``solimp`` attributes of the equality constraints.

``additional_xml`` *(default: "")*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to include additional XML in the URDF, you can specify the path to the file here. This file will be included in the produced XML.

.. note::

    Alternatively, ``additional_xml`` can be a list of files
