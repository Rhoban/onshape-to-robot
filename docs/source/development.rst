Development
===========

Run without installing
----------------------

When developing, you'll likely want to run the code with your changes without
building and installing the package. This can be done from the root of the
repository by using `python -m` to run the scripts as modules, e.g.:

.. code-block:: bash

    python -m onshape_to_robot.onshape_to_robot /path/to/dir/with/export/config

Run in a debugger
-----------------

Running in a debugger is an efficient way of tracing down defects. There are
several debuggers available for Python. To run a module in
`debugpy <https://github.com/microsoft/debugpy>`_, execute the following from
the root of the repository:

.. code-block:: bash

    python -m debugpy \
        --listen localhost:5678 \
        --wait-for-client \
        -m onshape_to_robot.onshape_to_robot \
       /path/to/dir/with/export/config
