
Adding dummy base link
======================

Introduction
------------

If enabling this processor, a dummy base called ``base_link`` will be added in your robot. The base will be attached to this link using a ``fixed`` joint.

``config.json`` entries
-----------------------

.. code-block:: javascript

    {
        // ...
        // General import options (see config.json documentation)
        // ...

        // Add a dummy base link (default: false)
        "add_dummy_base_link": true
    }

``add_dummy_base_link`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If set to ``true``, a dummy base link will be added to the robot. 