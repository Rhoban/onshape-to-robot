Use collisions as visual
========================

Introduction
------------

If this processor is enabled, the items from collision will be also used as visual items.

This can be used for:

* Debugging purpose, when you have no convenient way to visualize collisions in your downstream tools
* Creating a model that is lighter to load


``config.json`` entries
-----------------------

.. code-block:: javascript

    {
        // ...
        // General import options (see config.json documentation)
        // ...

        // Removes collision meshes
        "collisions_as_visual": true
    }

``collisions_as_visual`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If set to ``true``, collision will be used as visual.