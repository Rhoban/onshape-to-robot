Ball to Euler
=============

Introduction
------------

This processor turns the ``ball`` joints to three revolute joints. This can be convenient if your downstream simulator or tool can't handle ``ball`` joints.

``config.json`` entries
-----------------------

.. code-block:: javascript

    {
        // ...
        // General import options (see config.json documentation)
        // ...

        // Convert balls to Euler (default: false)
        "ball_to_euler": true,
        // Euler angles order (default: "xyz")
        "ball_to_euler_order": "xyz",
    }

``ball_to_euler`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If set to ``true``, all ball joints will be converted to Euler.

If you don't want to convert all your ball joints, you can specify a list of joints as follows:

.. code-block:: javascript

    {
        // Using specific joint lists, with wildcards
        "ball_to_euler": ["joint1", "shoulder_*"],
    }

``ball_to_euler_order`` *(default: "xyz")*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If set, the order of the euler angles will be set to the given value. The default is ``xyz``. 
The order should be ``xyz``, ``xzy``, ``zyx``, ``zxy``, ``yxz`` or ``yzx``.