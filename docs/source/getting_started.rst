Getting started
===============

Installing the package
----------------------

Run the following to install onshape-to-robot from `pypi <https://pypi.org/project/onshape-to-robot/>`_:

.. code-block:: bash

    pip install onshape-to-robot

.. _api-key:

Setting up your API key
-----------------------

You will need to obtain API key and secret from the
`My Account > Developper menu <https://cad.onshape.com/user/developer/apiKeys>`_

API key must be set as environment variables.

Using `.bashrc`
~~~~~~~~~~~~~~~

You can add something like this in your ``.bashrc``:

.. code-block:: bash

    # .bashrc
    # Obtained at https://dev-portal.onshape.com/keys
    export ONSHAPE_API=https://cad.onshape.com
    export ONSHAPE_ACCESS_KEY=Your_Access_Key
    export ONSHAPE_SECRET_KEY=Your_Secret_Key

Using `.env` file
~~~~~~~~~~~~~~~~~

Alternatively, you can also create a ``.env`` file in the root of your project:

.. code-block:: bash

    # .env
    # Obtained at https://dev-portal.onshape.com/keys
    ONSHAPE_API=https://cad.onshape.com
    ONSHAPE_ACCESS_KEY=Your_Access_Key
    ONSHAPE_SECRET_KEY=Your_Secret_Key

Setting up your export
----------------------

To export your own robot, first create a directory:

.. code-block:: bash

    mkdir my-robot

Then edit ``my-robot/config.json``, here is the minimum example:

.. code-block:: json

    {
        // Onshape URL of the assembly
        "url": "https://cad.onshape.com/documents/11a7f59e37f711d732274fca/w/7807518dc67487ad405722c8/e/5233c6445c575366a6cc0d50",
        // Output format
        "output_format": "urdf"
    }

.. note::

    The Onshape URL should be the one of your assembly. Be sure to be on the right tab when you copy it.

Once this is done, run the following command:

.. code-block:: bash

    onshape-to-robot my-robot


Testing your export
-------------------

You can test your export by running (PyBullet):

.. code-block:: bash

    onshape-to-robot-bullet my-robot

Or (MuJoCo):

.. code-block:: bash

    onshape-to-robot-mujoco my-robot

What's next ?
-------------

Before you can actually enjoy your export, you need to pay attention to the following:

* ``onshape-to-robot`` comes with some conventions to follow, in order to understand what in your robot is a degree of freedom, a link, a frame, etc. Make sure to read the :doc:`design-time considerations <design>`.
* There are some options you might want to specify in the :doc:`config.json <config>` file.
* Have a look at the `examples <https://github.com/rhoban/onshape-to-robot-examples>`_ available on GitHub.