# OnShape to Robot (SDF/URDF)

<p align="center">
<img src="img/main.png" />
</p>

This tool is based on the [OnShape API](https://dev-portal.onshape.com/) to retrieve
informations from an assembly and build an SDF or URDF model suitable for physics
simulation.

## Design-time considerations

There is some design constraints:

* Try to make your robot assembly mostly based on sub pre-assembled components (avoid to have a lot of constraints that are not relevant for the export). In this main assembly, do not use features such as sub-assemblies network.
* Degree of freedoms should be cylindrical or revolute mate connectors named `dof_something`, where `something` will be used to name the joint in the final document
* When doing this connection, click the children joint first. This will be used to find the trunk of the robot (part with children but no parent)

<p align="center">
<img src="img/smalls/design.png" />
</p>

It is possible to invert the axis for convenience by adding `_inv` at the end of the name. For instance
`dof_head_pitch_inv` will result in a joint named `head_pitch` having the axis inverted with the one
from the OnShape assembly.

## Requirements

First clone this repository:

    git clone git@github.com:Gregwar/onshape-urdf.git

Install the dependencies (can be in your python3 virtualenv):

    pip install numpy pybullet requests

You might also need OpenSCAD for pure shape estimation

    apt-get install openscad

## Configuration

Create your own robot configuration by copying the `robots/skeleton` directory
to your own:

    cp -R robots/skeleton/ robots/myrobot

Then edit `config.json` in your repository, here are the entries:

* `onshape_api`: URL for OnShape API
* `onshape_access_key` and `onshape_secret_key` are the API key you obtained
  from [OnShape developer portal](https://dev-portal.onshape.com/keys)
* `documentId` is the document ID to be imported (see above picture)
* `outputFormat` can be `sdf` or `urdf`
* `drawFrames` if you want the frames to be drawn
* `drawCollisions` if you want the elements from collisions to be also put
  in visuals instead of meshes (can be used to debug pure shapes)
* `useScads` if you want or not to use scad files for pure shapes (see below)
* Optionally, `assemblyName` can be used to specify the name of the assembly. Else
  the first assembly found in document will be used.
* `jointMaxEffort` and `jointMaxVelocity` can be used to specify the values that will
  be used in the `joints` entry. Alternatively, they can be dictionaries associating
  joints names to values.
* The `dynamics` key can be used to override inertial data computed by OnShape for
  a specific part (see example below)
* `noDynamics` can be set to `true` if you want to have all masses and inertia to 0
  (suppose you want to create an environment)
* `ignore` can be a list of part that you want to be ignored in the URDF export

Here is an example of configuration:

```json
{
    "onshape_api": "https://cad.onshape.com",
    "onshape_access_key": "[KEY]",
    "onshape_secret_key": "[SECRET]",

    "documentId": "483c803918afc4d52e2647f0",
    "assemblyName": "robot",
    "outputFormat": "urdf",
    "drawFrames": false,
    "drawCollisions": false,
    "useScads": true,
    "noDynamics": false,

    "jointMaxEffort": {
        "default": 1.5,
        "head_pitch": 0.5   
    },
    "jointMaxVelocity": 22,

    "dynamics": {
        "motorcase": {
            "mass": 0.5,
            "com": [0, 0.1, 0],
            "inertia": [0.1, 0, 0,
                        0, 0.1, 0,
                        0, 0, 0.1]
        }
    },

    "ignore": [
        "small_screw",
        "small_nut"
    ]
}
```

## Naming links

If you create a mate connector and name it `link_something`, the link corresponding to the part
on which it is attached will be named `something` in the resulting URDF.

## Running the import

You can run the import using:

    ./onshape-to-robot.py robots/myrobot

This will produce files in the directory (next to the `config.json` file), including STLs (mesh
files) and the `sdf` or `urdf`.

## Running the simulation

### PyBullet

To run the simulation:

    ./bullet.py robots/myrobot

The example code will send sinusoidal signals to motors target positions to see the robot
in action.

### Gazebo

You can give a try to gazebo using:

    gazebo
    gz model -m robot -d
    gz model --spawn-file=robot.sdf --model-name=robot

## Testing the example (quadruped robot)

You can give a try to the `demo-quadruped` robot, which is a public assembly that can
be [viewed here](https://cad.onshape.com/documents/11a7f59e37f711d732274fca/w/7807518dc67487ad405722c8/e/5233c6445c575366a6cc0d50):

    ./onshape-to-robot.py robots/demo-quadruped/

Note: parallel constraints are here to keep the robot in its "zero" position when exporting,
they can be supressed to manipulate the degrees of freedom in OnShape and unsupressed when
exporting, or just to reset the robot to the "zero" position.

## Pure shapes

By default, meshes are also used for collision. This is versatile but is computationally
expensive, and can be numerically instable.

You can approximate those parts with pure shapes (namely boxes, spheres and cylinders).
For this, we propose a solution based on `openscad`:

    apt-get install openscad

To do that, you need to create a `.scad` file next to `.stl` one. For instance `motor.scad`
that will approximate `motor.stl` file.

You can use `./edit-shape.py [stl-file]` that will automatically prepare and run the `.scad`
using a template visualizing the `.stl` with transparency, allowing you to edit the pure
shapes related:

<p align="center">
<img src="img/smalls/pure-shape.png" />
</p>

Then, the pure shapes from your scad will be used when generating the `sdf` or `urdf` file (next
time you will run `onshape-to-robot.py`, it will read your `.scad` files).

**Note: `center=true` is mandatory in the current version.**

Thus, if the `.scad` file is empty, your part will have no collision. If you want to use the mesh
again, simply remove the `.scad` file.

If you pass `drawCollisions` to `true`, the collisions will also be used for the render, which can
be useful to debug:

<p align="center">
<img src="img/smalls/shape-approx.png" />
</p>

## Frames

If you want to track some frames on your robot, you can do the following:

* Connect any part to your robot using mate relations in OnShape
* Name one of these relations `frame_something`, when `something` will be the name of
  the frame (a link) in the resulting `sdf` or `urdf`

<p align="center">
<img src="img/smalls/frame.png" />
</p>

If you want to give it a try, you can use the `bullet.py` in `urdf` mode, it will output the
frames on standard output.

## Cache

Some requests are cached for convenience (recovery of STL, massproperties etc.). You can run
the `clear-cache.sh` script to remove all cached requests.

## License

This project is under MIT License, read the `LICENSE` file for more information