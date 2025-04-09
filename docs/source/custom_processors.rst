.. _custom_processors:

Writing & registering custom Processor
======================================

Introduction
------------

In this documentation, you can find the description of all processors that are `registered by default <https://github.com/Rhoban/onshape-to-robot/blob/master/onshape_to_robot/processors.py>`_. You can also write your own processor, by writing a class and registering it in the ``config.json`` file as described below.


Minimal processor example
-------------------------

Below is a minimal example of processor you can write:

.. code-block:: python

    # my_project/my_custom_processor.py
    from onshape_to_robot.processor import Processor
    from onshape_to_robot.config import Config
    from onshape_to_robot.robot import Robot

    class MyCustomProcessor(Processor):
        def __init__(self, config: Config):
            super().__init__(config)

            self.use_my_custom: bool = config.get("use_my_custom", False)

        def process(self, robot: Robot):
            if self.use_my_custom:
                print(f"Custom processing for {robot.name} with custom processor.")

Remember that processors processes the robot intermediate representation, that you can find in `robot.py <https://github.com/Rhoban/onshape-to-robot/blob/master/onshape_to_robot/robot.py>`_.

Registering your processor(s)
-----------------------------

To register your processor, use the ``processors`` entry in ``config.json``:

.. code-block:: javascript

    {
        "processors": [
            // Custom processor
            "my_project.my_custom_processor:MyCustomProcessor",
            // Default processors
            "ProcessorScad",
            "ProcessorMergeParts",
            "ProcessorNoCollisionMeshes"
        ]
    }

.. note::

    The list of existing default processors can be found in `processors.py <https://github.com/Rhoban/onshape-to-robot/blob/master/onshape_to_robot/processors.py>`_.