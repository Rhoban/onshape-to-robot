Handling kinematic loops
========================

Some robots have *kinematic loops*, meaning that the kinematic chain is not a tree but a graph.
Here is an example:

Introduction
-------

Here is a 2D planar robot with kinematic loop, we assume the two first joints to be actuated and the others to
be passive:

.. raw:: html

    <center>
    <video width="50%" controls>
    <source src="https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/planar_2dof_trajectory.mp4" type="video/mp4">
    </video>
    </center>
    <br/>


Here, the two branches are connected together, thus this can't be represented as a tree.
URDF doesn't allow to directly represent this type of structure. We need to break it down in a tree, and enforce the
closing constraints in software during execution.
To enforce the constraints, frames can be placed at relevant positions, here is an example for the above robot:

.. image:: _static/img/opened_chain.png    
    :width: 200px
    :align: center

Using onshape-to-obot
---------------------

The above mentionned example can be achieved by adding :ref:`frames <custom-frames>`. However, this would require to
manually place two frames at each closing position. Onshape-to-robot comes with a more convenient way to achieve this,
by using mate connectors.

If you add a relation with ``closing_something`` as name, two frames will be added to your URDF
(``closing_something_1`` and ``closing_something_2``) that will be attached to the two parts mated.

This way, the closure is handled in onshape exactly the way it should appear in the final mechanism, and will result
in the two frames being placed at the correct position in the URDF.

Here is the complete `Onshape assembly <https://cad.onshape.com/documents/04b05c47de7576f35c0e99b3/w/68041f3f5c827a258b40039c/e/db543f501b01adf8144064e3?renderMode=0&uiState=665d8e3a6de6705d2f788204>`_ for the above example robot.
The onshape-to-robot export is available `here <https://github.com/Rhoban/placo-examples/tree/master/models/planar-2dof>`_.

Handling constraints on execution time
--------------------------------------

Here are some ressources on how to handle kinematic loops in software:

* In MuJoCo, you can add an `equality <https://mujoco.readthedocs.io/en/stable/computation/index.html#coequality>`_ constraint in your XML model.
* In `pyBullet <https://pybullet.org/wordpress/>`_, you can use `createConstraint` method to add the relevant constraint.
* In the `PlaCo <https://placo.readthedocs.io/>`_ solver, you can create a `RelativePositionTask`. See the `kinematics loop documentation section <https://placo.readthedocs.io/en/latest/kinematics/loop_closures.html>`_ for  more details. Some examples created with onshape-to-robot can be found in the `example gallery <https://placo.readthedocs.io/en/latest/kinematics/examples_gallery.html>`_.
