Onshape-to-robot documentation
==============================

.. raw:: html

    <center>
    <video width="70%" controls autoplay muted loop>
    <source src="https://github.com/Rhoban/onshape-to-robot-examples/raw/refs/heads/master/videos/sigmaban.mp4" type="video/mp4">
    </video>
    </center>
    <br/>


What is this ?
~~~~~~~~~~~~~~

.. image:: _static/img/main.png

``onshape-to-robot`` is a tool that allows you to export robots designed from the **Onshape CAD** software
to descriptions format like **URDF**, **SDF** or **MuJoCo**, so that you can use them for physics simulation or in your running code
(requesting frames, computing dynamics etc.)

* `onshape-to-robot GitHub repository <https://github.com/rhoban/onshape-to-robot/>`_
* `Robots examples GitHub repository <https://github.com/rhoban/onshape-to-robot-examples/>`_
* `onshape-to-robot on pypi <https://pypi.org/project/onshape-to-robot/>`_

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   getting_started
   design
   config
   exporter_urdf
   exporter_sdf
   exporter_mujoco
   kinematic_loops
   processors
   cache
