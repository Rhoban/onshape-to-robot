Convex decomposition (CoACD)
============================

Introduction
------------

If this processor is enabled, the collision meshes will be decomposed using `CoACD <https://github.com/SarahWeiii/CoACD>`_ convex decomposition.


``config.json`` entries
-----------------------

.. code-block:: javascript

    {
        // ...
        // General import options (see config.json documentation)
        // ...

        // Enables convex decomposition
        "convex_decomposition": true
        // Use Rainbow colors instead of the part color
        "rainbow_colors": true
    }

``convex_decomposition`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If set to ``true``, collision meshes will be decomposed using CoACD.

``rainbow_colors`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If set to ``true``, the collision meshes will be colored using rainbow colors instead of the part color.