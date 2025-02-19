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
`Onshape developer portal <https://dev-portal.onshape.com/keys>`_

We recommend you to store your API key and secret in environment variables, you can add something
like this in your ``.bashrc``:

.. code-block:: bash

    # Obtained at https://dev-portal.onshape.com/keys
    export ONSHAPE_API=https://cad.onshape.com
    export ONSHAPE_ACCESS_KEY=Your_Access_Key
    export ONSHAPE_SECRET_KEY=Your_Secret_Key

Setting up your export
----------------------

To export your own robot, first create a directory:

.. code-block:: bash

    mkdir my-robot

Then edit ``my-robot/config.json``, here is the minimum example:

.. code-block:: json

    {
        // Id of the Onshape document (see below)
        "document_id": "document-id",
        // Output format
        "output_format": "urdf"
    }

The ``document-id`` is the token (below XXXXXXXXX) you can find in Onshape URL when editing your robot:

.. code-block:: bash

    https://cad.onshape.com/documents/XXXXXXXXX/w/YYYYYYYY/e/ZZZZZZZZ
                                      ^^^^^^^^^
                                This is the document id

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

Head's up !
-----------

Before you can actually enjoy your export, you need to pay attention to the following:

* ``onshape-to-robot`` comes with some conventions to follow, in order to understand what in your robot is a degree of freedom, a link, a frame, etc. Make sure to read the :doc:`design-time considerations <design>`.
* There are some options you might want to specify in the :doc:`config.json <config>` file.