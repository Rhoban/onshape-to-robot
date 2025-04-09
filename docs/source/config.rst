Configuration (config.json)
===========================

Specific entries
----------------

Below are the global configuration entries.
You might also want to check out the following documentation for more specific entries:

* Exporters
    * :doc:`URDF specific entries <exporter_urdf>`
    * :doc:`SDF specific entries <exporter_sdf>`
    * :doc:`MuJoCo specific entries <exporter_mujoco>`
* :doc:`Processors <processors>` can define their own specific entries


``config.json`` entries
-----------------------

Here is an example of complete ``config.json`` file, with details below:

.. code-block:: javascript

    // config.json general options
    // for urdf or mujoco specific options, see documentation
    {
        // Onshape assembly URL
        "url": "https://cad.onshape.com/documents/11a7f59e37f711d732274fca/w/7807518dc67487ad405722c8/e/5233c6445c575366a6cc0d50",
        // Output format: urdf or mujoco (required)
        "output_format": "urdf",
        // Output filename (default: "robot")
        // Extension (.urdf, .xml) will be added automatically
        "output_filename": "robot",
        // Assets directory (default: "assets")
        "assets_dir": "assets",

        // If you don't use "url", you can alternatively specify the following
        // The Onshape document id to parse, see "getting started" (optional)
        "document_id": "document-id",
        // The document version id (optional)
        "version_id": "version-id",
        // The workspace id (optional) 
        "workspace_id": "workspace-id",
        // Element id (optional)
        "element_id": "element-id",
        // Assembly name to use in the document (optional)
        "assembly_name": "robot",

        // Onshape configuration to use (default: "default")
        "configuration": "Configuration=BigFoot;RodLength=50mm",
        // Robot name (default: "onshape")
        "robot_name": "robot",

        // Ignore limits (default: false)
        "ignore_limits": true,

        // Parts to ignore (default: {})
        "ignore": {
            // Ignore visual for visual
            "part1": "visual",
            "screw*": "visual",

            // Ignore everything expect "leg" for collision
            "*" : "collision"
            "!leg": "collision"
        },

        // Whether to keep frame links (default: false)
        "draw_frames": true,
        // Override the color of all links (default: None)
        "color": [0.5, 0.1, 0.1],

        // Disable dynamics retrieval (default: false)
        "no_dynamics": true,

        // Post import commands (default: [])
        "post_import_commands" [
            "echo 'Import done'",
            "echo 'Do something else'"
        ],

        // Custom processors
        "processors": [
            "my_project.my_custom_processor:MyCustomProcessor"
        ]

        // More options available in specific exporters (URDF, SDF, MuJoCo)
        // More options available in processors
    }

.. note::

    Comments are supported in the ``config.json`` file.

.. note::

    Since ``1.0.0``, all configuration entries are now snake case. For backward compatibility reasons, the old
    camel case entries are still supported. (for example, ``document_id`` and ``documentId`` are equivalent).

``url`` *(required)*
~~~~~~~~~~~~~~~~~~~~

The Onshape URL of the assembly to be exported. Be sure you are on the correct tab when copying the URL.

``output_format`` *(required)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**required**

This should be either ``urdf`` or ``mujoco`` to specify which output format is wanted for robot description
created by the export.

``output_filename`` *(default: robot)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is the name of the output file without extension. By default "robot" (for example: ``robot.urdf``, ``robot.sdf`` or ``robot.xml``).

``assets_dir`` *(default: "assets")*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is the directory where the assets (like meshes) will be stored.

``assembly_name`` *(optional)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This can be used to specify the name of the assembly (in the Onshape document) to be used for robot export.

If this is not provided, ``onshape-to-robot`` will list the assemblies. If more than one assembly is found,
an error will be raised.

``document_id`` *(optional)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you don't specify the URL, this is the onshape ID of the document to be imported. It can be found in the Onshape URL,
just after ``document/``.

.. code-block:: bash

    https://cad.onshape.com/documents/XXXXXXXXX/w/YYYYYYYY/e/ZZZZZZZZ
                                      ^^^^^^^^^
                                This is the document id

``version_id`` *(optional)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you don't specify the URL, this argument can be used to use a specific version of the document instead of the last one. The version ID
can be found in URL, after the ``/v/`` part when selecting a specific version in the tree.

If it is not specified, the workspace will be retrieved and the live version will be used.

``workspace_id`` *(optional)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you don't specify the URL, this argument can be used to use a specific workspace of the document. This can be used for specific branches
ofr your robot without making a version.
The workspace ID can be found in URL, after the ``/w/`` part when selecting a specific version in the tree.

``element_id`` *(optional)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you don't specify the URL, this argument can be used to use a specific element of the document.
The element ID can be found in URL, after the ``/e/`` part when selecting a specific version in the tree.

``configuration`` *(default: "default")*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is the robot configuration string that will be passed to Onshape. Lists, booleans and quantities are allowed. For example:

.. image:: _static/img/configuration.png
    :width: 300px
    :align: center

Should be written as the following:

.. code-block:: text

    Configuration=Long;RemovePart=true;Length=30mm

.. note::

    Alternatively, you can specify the configuration as a dictionary:

    .. code-block:: json

        {
            // ...
            "configuration": {
                "Configuration": "Long",
                "RemovePart": true,
                "Length": "30mm"
            }
        }


``robot_name`` *(default: "dirname")*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Specifies the robot name. This value is typically present in the header of the exported files.

If it is not specified, the directory name will be used.

``ignore_limits`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If set to ``true``, the joint limits coming from Onshape will be ignored during export.

``ignore`` *(default: {})*
~~~~~~~~~~~~~~~~~~~~~~~~~~

This can be a list of parts that you want to be ignored during the export.

Alternatively, you can use a dict, where the values are either ``all``, ``visual`` or ``collision``. The rules will apply in order of appearance.

You can use wildcards ``*`` to match multiple parts.

You can prefix the part name with ``!`` to exclude it from the rule. For example, the following will ignore all parts for visual, except the ``leg`` part, turning the ignore list to a whitelist:

.. code-block:: json

    {
        // Ignore everything from visual
        "*": "collision",
        // Except the leg part
        "!leg": "collision"
    }

.. note::

    The dynamics of the part will not be ignored, but the visual and collision aspect will.

.. _draw-frames:

``draw_frames`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When , the part that is used for positionning the frame is
by default excluded from the output description (a dummy link is kept instead). Passing this option to ``true`` will
keep it instead.

``no_dynamics`` *(default: false)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This flag can be set if there is no dynamics. In that case all masses and inertia will be set to 0.
In pyBullet, this will result in static object (think of some environment for example).


``color`` *(default: None)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Can override the color for parts (should be an array: ``[r, g, b]`` with numbers from 0 to 1)

``post_import_commands`` *(default: [])*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is an array of commands that will be executed after the import is done. It can be used to be sure that
some processing scripts are run everytime you run onshape-to-robot.

``processors`` *(default: None)*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

See :ref:`custom processors <custom_processors>` for more information.