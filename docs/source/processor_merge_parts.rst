.. _processor-merge-parts:

Merge STLs
==========

Introduction
------------

This processor merge the meshes of all parts in the links into a single one.

``config.json`` entries
-----------------------

.. code-block:: javascript

    {
        // ...
        // General import options (see config.json documentation)
        // ...

        // Merge STL meshes (default: false)
        "merge_stls": true,
    }

``merge_stls`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If set to ``true``, each link parts meshes will be merged in a single one. The resulting part will be named after the link.

You can also set it to ``"visual"`` to merge only the visual parts, or to ``"collision"`` to merge only the collision parts.

Alternatively, ``merge_stls`` can also be an object as shown below, allowing filtering:

.. code-block:: javascript

    {
        // ...
        "merge_stls": {
            // Everything is merge by default
            "*": "both",
            // Feet will have collisions merged only
            "foot_*": "collision",
            // The trunk will not be merged at all (excluded with !)
            "!trunk": "both"
        }
    }

Values should be ``both``, ``visual`` or ``collision``. Wildcard patterns are supported, the last matching pattern is used. The ``!`` prefix can be used to apply exclusion rules.