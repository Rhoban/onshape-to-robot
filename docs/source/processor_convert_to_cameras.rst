.. _processor-convert-to-cameras:

Convert to Cameras
==================

Introduction
------------

This processor converts frames to camera elements in your MuJoCo model. It is enabled by default via the ``ProcessorConvertToCameras`` processor.

Frames are automatically matched to camera names using the ``cameras`` configuration entry, and the resulting ``<camera>`` elements are added to the appropriate body tags in the MuJoCo XML output.

``config.json`` entries
-----------------------

.. code-block:: javascript

    {
        // ...
        // General import options (see config.json documentation)
        // ...

        "cameras": {
            "wrist_camera": "wrist_camera_frame",
            "head_camera": "head_camera_frame"
        }
    }

``cameras`` *(default: {})*
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Maps camera names to frame names defined in your Onshape assembly. The keys are the camera names that will appear in the MuJoCo XML, and the values are the corresponding frame names.

To add cameras to your robot:

1. Define camera frame references in your Onshape assembly using mate connectors
2. Configure camera mappings in ``config.json``

.. important::

    **Camera Frame Orientation in MuJoCo**

    When creating mate connectors in Onshape for cameras, ensure the frame orientation follows MuJoCo's camera convention:

    * The camera looks along the **-Z axis** (negative Z direction)
    * The **+X axis** points to the right
    * The **+Y axis** points up

    This means your mate connector's Z-axis should point **into** the camera (opposite of the viewing direction).

Generated MuJoCo XML
~~~~~~~~~~~~~~~~~~~~~

Cameras will be exported as ``<camera>`` elements within the appropriate body tags:

.. code-block:: xml

    <body name="wrist_link" ...>
        <!-- ... body content ... -->

        <!-- Camera wrist_camera -->
        <camera name="wrist_camera" pos="0 0.04 0" quat="0 0 1 0" fovy="45" mode="fixed" resolution="640 480" />
    </body>