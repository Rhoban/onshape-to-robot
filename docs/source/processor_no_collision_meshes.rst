Removing collision meshes
=========================

Introduction
------------

This processor ensure no collision meshes are rendered.

.. note::

    Alternatively, you can use ignore lists to ignore meshes from being processed:

    .. code-block:: json

        {
            "ignore": {
                "*": "collision"
            }
        }

    However, this will prevent the meshes from being available to other processors (for example, to be approximated with pure shapes).


``config.json`` entries
-----------------------

.. code-block:: javascript

    {
        // ...
        // General import options (see config.json documentation)
        // ...

        // Removes collision meshes
        "no_collision_meshes": true
    }

``no_collision_meshes`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If set to ``true``, collision meshes will be removed.