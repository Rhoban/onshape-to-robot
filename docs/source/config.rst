Export your own robot (writing config.json)
===========================================

To export your own robot, first create a directory:

.. code-block:: bash

    mkdir my-robot

Then edit ``my-robot/config.json``, here is the minimum example:

.. code-block:: json

    {
        "documentId": "document-id",
        "outputFormat": "urdf"
    }

The ``document-id`` is the number (below XXXXXXXXX) you can find in Onshape URL:

.. code-block:: bash

    https://cad.onshape.com/documents/XXXXXXXXX/w/YYYYYYYY/e/ZZZZZZZZ

Once this is done, if you properly :doc:`installed and setup your API key <installation>`, just run:

.. code-block:: bash

    onshape-to-robot my-robot

``config.json`` entries
-----------------------

Here is the full list of possible entries for this configuration.

``onshape_api``, ``onshape_access_key`` and ``onshape_secret_key``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Those allow you to specify the URL for onshape API (``onshape_api``), and your
API key (``onshape_access_key``) and secret key (``onshape_secret_key``).

.. warning::

    Even if this is possible to specify the api key in the ``config.json`` file, it is
    very sensitive and we recommend you to set them up as environment variable.
    See the following section: :ref:`Setting up API key section <api-key>`.

``documentId``
~~~~~~~~~~~~~~

This is the onshape ID of the document to be imported. It can be found in the Onshape URL,
just after ``document/``.


``outputFormat``
~~~~~~~~~~~~~~~~

**required**

This should be either ``urdf`` or ``sdf`` to specify which output format is wanted for robot description
created by the export.

``assemblyName``
~~~~~~~~~~~~~~~~

*optional*

This can be used to specify the name of the assembly (in the Onshape document) to be used for robot export. If none
is used, the first assembly found will be used.

``workspaceId``
~~~~~~~~~~~~~~~

*optional, no default*

This argument can be used to use a specific workspace of the document. This can be used for specific branches
ofr your robot without making a version.
The workspace ID can be found in URL, after the ``/w/`` part when selecting a specific version in the tree.

``versionId``
~~~~~~~~~~~~~

*optional, no default*

This argument can be used to use a specific version of the document instead of the last one. The version ID
can be found in URL, after the ``/v/`` part when selecting a specific version in the tree.

If it is not specified, the very last version will be used for import.

``configuration``
~~~~~~~~~~~~~~~~~

*optional, default: "default"*

This is the robot configuration string that will be passed to Onshape. An example of format:

.. code-block:: js

    left_motor_angle=3+radian;enable_yaw=true

``drawFrames``
~~~~~~~~~~~~~~

*optional, default: false*

When :ref:`adding custom frames to your model <custom-frames>`, the part that is used for positionning the frame is
by default excluded from the output description (a dummy link is kept instead). Passing this option to ``true`` will
keep it instead.

``drawCollisions``
~~~~~~~~~~~~~~~~~~

*optional, default: false*

If you use :doc:`pure shapes approximations <pure-shapes>`, the collisions in your description will not be meshes
but shapes like boxes, cylinders etc. If you pass this argument to ``true``, it will use the same output in the
``visual`` tag, making the visual similar to what is used for collisions.

This can be used for debugging, but also to lighten the robot visualization if it is complex during experiments
and avoiding loading meshes just for visualization.

``useScads``
~~~~~~~~~~~~

*optional, default: true (needs openscad installed)*

If you create :doc:`pure shapes approximations <pure-shapes>` of your parts, you will have ``.scad`` files sitting
in your directory, this flag can be used to disable using them (if ``false``, full meshes will be then used for
collisions).

``pureShapeDilatation``
~~~~~~~~~~~~~~~~~~~~~~~

*optional, default: 0*

If you want to use pure shape as safety check for collisions, you can use this parameter to add some extra
dilatation to all of them.

``jointMaxEffort`` and ``jointMaxVelocity``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

*optional, default: 1 and 20*

Those parameters can be used to specify the values that will be included in the ``joint`` entries.

Alternatively, they can be dictionaries associating named joints to the values.


``dynamics``
~~~~~~~~~~~~

*optional, default: {}*

This ``dict`` can be used to override the mass and inertia computed by Onshape for a specific part.
See :ref:`example <example-config>` below.


``noDynamics``
~~~~~~~~~~~~~~

*optional, default: false*

This flag can be set if there is no dynamics. In that case all masses and inertia will be set to 0.
In pyBullet, this will result in static object (think of some environment for example).

``ignore``
~~~~~~~~~~

*optional, default: []*

This can be a list of parts that you want to be ignored during the export.

Note: the dynamics of the part will not be ignored, but the visual and collision aspect will.

``whitelist``
~~~~~~~~~~~~~

*optional, default: None*

This can be used as the opposed of ``ignore``, to import only some items listed in the configuration
(all items not listed in ``whitelist`` will be ignored if it is not ``None``)

``color``
~~~~~~~~~

*optional, default: None*

Can override the color for parts (should be an array: ``[r, g, b]`` with numbers from 0 to 1)

``packageName``
~~~~~~~~~~~~~~~

*optional*

Prepends a string to the paths of STL files. This is helpful for ROS users as they often need to specify their
``robot_description`` package.

``addDummyBaseLink``
~~~~~~~~~~~~~~~~~~~~

*optional*

Adds a ``base_link`` without inertia as root. This is often necessary for ROS users.

``robotName``
~~~~~~~~~~~~~

*optional*

Specifies the robot name.

``additionalXML``
~~~~~~~~~~~~~~~~~

*optional*

Specifies a file with XML content that is inserted into the URDF/SDF at the end of the file. Useful to add things that can't be modelled in onshape, e.g. simulated sensors.

``useFixedLinks``
~~~~~~~~~~~~~~~~~

*optional, default: false*

With this option, visual parts will be added through fixed links to each part of the robot. Mostly, this feature
is a hack to keep colors properly for rendering in PyBullet (see https://github.com/bulletphysics/bullet3/issues/2650).

``mergeSTLs``
~~~~~~~~~~~~~

*optional, default: "no"*

Can be "no", "visual", "collision" or "all".

This can be used to merge STLs file of the same ``link`` into one unique STL. It is actually better combined with
``simplifySTLs``, that can be used to reduce the STL file sizes.

**Note: this will only merge visual for visual, see ``mergeSTLsCollisions``**

``mergeSTLsCollisions``
~~~~~~~~~~~~~~~~~~~~~~~

*optional, default: false*

STLs used for collisions will also be merged if this flag is ``true``. 

``simplifySTLs``
~~~~~~~~~~~~~~~~

*optional, default: "no"*

Can be "no", "visual", "collision" or "all".

If this is set, the STL files will be reduced (see ``maxSTLSize``). This requires ``meshlab`` tool (``sudo
apt-get install meshlab``).

``maxSTLSize``
~~~~~~~~~~~~~~

*optional, default: 3*

This is the maximum size (in ``M``) of STL files before they are reduced by ``simplifySTLs``.

``useCollisionsConfigurations``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

*optional, default: true*

With this option (enabled by default), the collisions=true configuration will be passed when exporting STL
meshes (and NOT dynamics), in order to retrieve simplified mesh parts from OnShape.

This is a way to approximate your robot with simpler meshes.

``postImportCommands``
~~~~~~~~~~~~~~~~~~~~~~

*optional, default: []*

This is an array of commands that will be executed after the import is done. It can be used to be sure that
some processing scripts are run everytime you run onshape-to-robot.

.. _example-config:

Example ``config.json`` file
----------------------------

Here is an example of configuration:

.. code-block:: js

    {
        // You should store those three in environment variables
        "onshape_api": "https://cad.onshape.com",
        "onshape_access_key": "[KEY]",
        "onshape_secret_key": "[SECRET]",

        // Can be found in the URL when editing the assembly
        "documentId": "483c803918afc4d52e2647f0",
        // If not specified, the first assembly will be used
        "assemblyName": "robot",
        // Can be urdf or sdf
        "outputFormat": "urdf",
        // The frames parts are kept in the final file
        "drawFrames": false,
        // Collisions (pure shapes) are also used in the visual section
        "drawCollisions": false,
        // Wether or not the scan for SCAD files (pure shapes) should be done
        "useScads": true,
        // Masses, com and inertias will be zero (can be used if you import a static
        // field for example)
        "noDynamics": false,
        // Should the STLs of the same link be merged?
        "mergeSTLs": "no",
        // Should we simplify STLs files?
        "simplifySTLs": "no",
        // Maximum size (M) of STL files to run simplification (required meshlab)
        "maxSTLSize": 3,

        // Those can be used to configure the joint max efforts and velocity, and
        // overriden for specific joints
        "jointMaxEffort": {
            "default": 1.5,
            "head_pitch": 0.5
        },
        "jointMaxVelocity": 22,

        // This can be used to override the dynamics of some part (suppose it's a compound
        // which dynamics is well specified)
        "dynamics": {
            "motorcase": {
                "mass": 0.5,
                "com": [0, 0.1, 0],
                "inertia": [0.1, 0, 0,
                            0, 0.1, 0,
                            0, 0, 0.1]
            },
            // "fixed" can be used to assign a null mass to the object, which makes it fixed (non-dynamics)
            "base": "fixed"
        },

        // Some parts can be totally ignored during import
        "ignore": [
            "small_screw",
            "small_nut"
        ]
    }

Testing your robot in simulator
-------------------------------

You can then use the ``onshape-to-robot-bullet my-robot`` command to give a try to your robot.
