.. _exporter-mujoco:

MuJoCo
======

Introduction
-------------

MuJoCo is a standard physics simulator, coming with an extensive description format.

* Frames will be added as ``site`` tags in the MuJoCo XML file.
* *Cameras* can be added using custom processors (see :ref:`Cameras <cameras>` section below).
* *Actuators* will be created for all actuated joints (see below).
* When :ref:`kinematic loops <kinematic-loops>` are present, they will be enforced using equality constraints.

  * If the loop is achieved using a ``fixed`` connector, a ``weld`` constraint will be added.
  * If a ``ball`` joint is used, a ``connect`` constraint will be added
  * If a ``revolute`` joint is used, two ``connect`` constraints will be used

* Additionally to the ``robot.xml`` file, a ``scene.xml`` file will be produced, adding floor and lighting useful for testing purpose.

``config.json`` entries (MuJoCo)
--------------------------------

Here is an example of complete ``config.json`` file, with details below:

.. code-block:: javascript

    {
        "url": "document-url",
        "output_format": "mujoco",
        // ...
        // General import options (see config.json documentation)
        // ...

        // Additional XML file to be included in the URDF (default: "")
        "additional_xml": "my_custom_file.xml",

        // Override joint properties (default: {})
        "joint_properties": {
            // Default properties for all joints
            "*": {
                "actuated": true,
                "forcerange": 10.0,
                "frictionloss": 0.5,
                "limits": [0.5, 1.2]
                // ...
            },
            // Set the properties for a specific joint
            "joint_name": {
                "forcerange": 20.0,
                "frictionloss": 0.1
                // ...
            }
        },

        // Override equality attributes
        "equalities": {
            "closing_branch*": {
                "solref": "0.002 1",
                "solimp": "0.99 0.999 0.0005 0.5 2"
            }
        }
    }

``joint_properties`` *(default: {})*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Allow to specify the properties of the joints produced in the URDF output. The key should be joint names. The special ``default`` key will set default values for each joints.

Possible values are:

* ``actuated``: *(default: true)* whether an actuator should be associated to this joint,
* ``class``: a ``class="..."`` to be added to the joint (and actuator) 
* ``type`` *(default: position)* defines the actuator that will be produced
* ``range`` *(default: true)*: if ``true``, the joint limits are reflected on the joint ``range`` attribute
* ``limits``: Override the joint limits, should be a list of two values (min, max)

* The following are reflected as ``<joint ...>`` attributes:

    * ``frictionloss``
    * ``damping``
    * ``armature``
    * ``stiffness``

* The following are reflected as actuator (``<position ...>`` or other) attributes:

    * ``kp``, ``kv`` and ``dampratio`` gains
    * ``forcerange``

``equalities`` *(default: {})*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This entry allows to override the equality attributes of the MuJoCo XML file. The key should be the equality name (which might contains wildcards ``*``), and the value should be a dictionary of attributes.

This can be used to adjust the ``solref`` and ``solimp`` attributes of the equality constraints.

``additional_xml`` *(default: "")*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to include additional XML in the URDF, you can specify the path to the file here. This file will be included in the produced XML.

.. note::

    Alternatively, ``additional_xml`` can be a list of files

.. _cameras:

Cameras
-------

Camera support allows you to add camera elements to your MuJoCo model. Cameras are typically added through custom processors that convert frame references to camera elements.

Basic camera usage
~~~~~~~~~~~~~~~~~~

To add cameras to your robot, you need to:

1. Define camera frame references in your Onshape assembly using mate connectors
2. Configure camera mappings in ``config.json``
3. Use a custom processor to convert frames to cameras

.. important::

    **Camera Frame Orientation in MuJoCo**

    When creating mate connectors in Onshape for cameras, ensure the frame orientation follows MuJoCo's camera convention:

    * The camera looks along the **-Z axis** (negative Z direction)
    * The **+X axis** points to the right
    * The **+Y axis** points up

    This means your mate connector's Z-axis should point **into** the camera (opposite of the viewing direction).

Example ``config.json`` entry:

.. code-block:: javascript

    {
        "cameras": {
            "wrist_camera": "wrist_camera_frame",
            "head_camera": "head_camera_frame"
        },
        "processors": [
            "my_processors.convert_to_cameras:ProcessorConvertToCameras",
            // ... other processors
        ]
    }

Camera processor example
~~~~~~~~~~~~~~~~~~~~~~~~~

Here's an example processor that converts frames to cameras:

.. code-block:: python

    from onshape_to_robot.processor import Processor
    from onshape_to_robot.config import Config
    from onshape_to_robot.robot import Robot, Camera

    class ProcessorConvertToCameras(Processor):
        """Convert frames to camera elements."""

        def __init__(self, config: Config):
            super().__init__(config)
            self.cameras = config.get("cameras", {})

        def process(self, robot: Robot):
            for camera_name, frame_name in self.cameras.items():
                for link in robot.links:
                    if frame_name in link.frames:
                        print(f"Creating camera '{camera_name}' from frame '{frame_name}'")
                        robot.cameras.append(
                            Camera(camera_name, link.name, link.frames.pop(frame_name))
                        )
                        break

Camera properties
~~~~~~~~~~~~~~~~~

The ``Camera`` class supports the following optional properties:

* ``fovy`` *(default: 45.0)*: Vertical field of view in degrees
* ``mode`` *(default: "fixed")*: Camera mode (``fixed``, ``track``, etc.)
* ``resolution`` *(default: (640, 480))*: Image resolution as a tuple

Example with custom properties:

.. code-block:: python

    from onshape_to_robot.robot import Camera

    camera = Camera(
        name="my_camera",
        link_name="head_link",
        T_link_camera=transform_matrix,
        fovy=60.0,
        mode="fixed",
        resolution=(1280, 720)
    )
    robot.cameras.append(camera)

Generated MuJoCo XML
~~~~~~~~~~~~~~~~~~~~

Cameras will be exported as ``<camera>`` elements within the appropriate body tags:

.. code-block:: xml

    <body name="wrist_link" ...>
        <!-- ... body content ... -->

        <!-- Camera wrist_camera -->
        <camera name="wrist_camera" pos="0 0.04 0" quat="0 0 1 0" fovy="45" mode="fixed" resolution="640 480" />
    </body>