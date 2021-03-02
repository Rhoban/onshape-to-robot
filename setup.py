import setuptools

with open("README-pypi.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="onshape-to-robot",
    version="0.3.9",
    author="Rhoban team",
    author_email="team@rhoban.com",
    description="Converting OnShape assembly to robot definition (SDF or URDF) through OnShape API ",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/rhoban/onshape-to-robot/",
    packages=setuptools.find_packages(),
    scripts=['onshape-to-robot', 'onshape-to-robot-bullet', 
    'onshape-to-robot-edit-shape', 'onshape-to-robot-clear-cache', 'onshape-to-robot-pure-sketch'],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    keywords="robot robotics cad design onshape bullet pybullet sdf urdf gazebo ros model kinematics",
    install_requires=[
        "numpy", "pybullet", "requests", "commentjson", "colorama", "numpy-stl", "transforms3d"
    ],
    include_package_data=True,
    package_data={'': ['bullet/*', 'README.md']},
    python_requires='>=3.6',
)
