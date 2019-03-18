# OnShape to URDF importer

## Requirements

First clone this repository:

    git clone git@github.com:Gregwar/onshape-urdf.git

Install the dependencies (can be in your python3 virtualenv):

    pip install numpy pybullet requests

You might also need OpenSCAD for pure shape estimation

    apt-get install openscad

## Configuration

Copy `config.json.example` to `config.json` and edit it.

You should get an API key for your account on the
[OnShape API panel](https://dev-portal.onshape.com/keys) to
fill the `onshape_access_key` and `onshape_secret_key`.

XXX Todo other parameters

## Design constaints

XXX Todo

## Running the import

You can run the import using:

    python urdf.py

## Running the simulation to test

To run the simulation:

    python bullet.py

## Pure shapes

XXX TODO