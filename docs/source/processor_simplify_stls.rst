Simplify STLs
=============

Introduction
------------

This processor will simplify the STLs files so that their size don't exceed a predefined limit.
Used with :ref:`processor-merge-parts`, it will simplify the merged STLs.

Requirements
------------

For this processor to work, ensure pymeshlab is installed:

.. code-block:: bash

    pip install pymeshlab

``config.json`` entries
-----------------------

.. code-block:: javascript

    {
        // ...
        // General import options (see config.json documentation)
        // ...

        // Simplify STL meshes (default: false)
        "simplify_stls": true,
        // Maximum size of the STL files in MB (default: 3)
        "max_stl_size": 1
    }

``simplify_stls`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If set to ``true``, the STL files will be simplified.

``max_stl_size`` *(default: 3)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The maximum size of the STL files in MB. If the size of a file exceeds this limit, it will be simplified.