
Installation & requirements
===========================

Requirements
-------------

You will need an Onshape account and Python 3.

.. note::

    You might also need OpenSCAD for pure shape estimation and meshlab for STLs simplification. Those are not
    mandatory requirements but will unlock more features:

    .. code-block:: bash

        apt-get install openscad meshlab

Installation
------------

From pip
~~~~~~~~

Run the following to install `onshape-to-robot from pypi <https://pypi.org/project/onshape-to-robot/>`_:

.. code-block:: bash

    pip install onshape-to-robot

From source repository
~~~~~~~~~~~~~~~~~~~~~~

First, clone the repository:

.. code-block:: bash

    git clone git@github.com:Rhoban/onshape-to-robot.git

Install the dependencies (can be in your python3 virtualenv):

.. code-block:: bash

    pip install numpy pybullet requests commentjson colorama numpy-stl

You can know use the scripts that are in the root folder of repository (feel free to add it to your
``$PATH`` and ``$PYTHONPATH`` to run it from anywhere)

.. _api-key:

Setting up your API key
-----------------------

To go any further, you will need to obtain API key and secret from the
`OnShape developer portal <https://dev-portal.onshape.com/keys>`_

We recommend you to store your API key and secret in environment variables, you can add something
like this in your ``.bashrc``:

.. code-block:: bash

    // Obtained at https://dev-portal.onshape.com/keys
    export ONSHAPE_API=https://cad.onshape.com
    export ONSHAPE_ACCESS_KEY=Your_Access_Key
    export ONSHAPE_SECRET_KEY=Your_Secret_Key

Alternatively, those keys can be stored in the ``config.json`` file, that will override those
parameters (see below). It is however preferred to use environment variables because you can then
share safely your `config.json` without sharing your secret keys.