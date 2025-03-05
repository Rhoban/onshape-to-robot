Using fixed links
=================

Introduction
------------

If this processor is enabled, all parts will be separated in a link, associated with its parent using a ``fixed`` link.

.. note::

    Doing this is likely to result in poor performance in physics engine, but can be useful for debugging.

``config.json`` entries
-----------------------

.. code-block:: javascript

    {
        // ...
        // General import options (see config.json documentation)
        // ...

        // Adding fixed links, resulting in one link per part
        "use_fixed_links": true
    }

``use_fixed_links`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If set to ``true``, a dummy base link will be added to the robot. 

Alternatively, ``use_fixed_links`` can be a list of the links for which you want the fixed links to be added.