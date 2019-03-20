# OnShape to SDF/URDF importer

![](img/main.png)

This tool is based on the [OnShape API](https://dev-portal.onshape.com/) to retrieve
informations from an assembly and build an SDF or URDF model suitable for physics
simulation.

## Design-time considerations

There is some design constraints:

* Try to make your robot assembly mostly based on sub pre-assembled components (avoid to have a lot of constraints that are not relevant for the export)
* Degree of freedoms should be cylindrical mate connectors named `dof_something`, where `something` will be used to name the joint in the final document
* When doing this connection, click the children joint first. This will be used to find the trunk of the robot (part with children but no parent)

![](img/design.png)

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
  from [OnShape developper portal](https://dev-portal.onshape.com/keys)
* `documentId` is the document ID to be imported (see above picture)
* `outputFormat` can be `sdf` or `urdf`
* `drawFrames` if you want the frames to be drawn
* `drawCollisions` if you want the elements from collisions to be also put
  in visuals instead of meshes (can be used to debug pure shapes)
* `useScads` if you want or not to use scad files for pure shapes (see below)
* Optionnaly, `assemblyName` can be used to specify the name of the assembly. Else
  the first assembly found in document will be used.

## Running the import

You can run the import using:

    ./run-import.py robots/myrobot

## Running the simulation to test

### PyBullet

To run the simulation:

    ./bullet.py robots/myrobot

### Gazebo

You can give a try to gazebo using:

    gazebo
    gz model -m robot -d
    gz model --spawn-file=robot.sdf --model-name=robot


## Pure shapes