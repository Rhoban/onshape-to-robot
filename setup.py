import setuptools

with open("README-pypi.md", "r", encoding="utf-8") as stream:
    long_description = stream.read()

setuptools.setup(
    name="onshape-to-robot",
    version="0.3.17",
    author="Rhoban team",
    author_email="team@rhoban.com",
    description="Converting OnShape assembly to robot definition (SDF or URDF) through OnShape API ",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/rhoban/onshape-to-robot/",
    packages=setuptools.find_packages(),
    entry_points={
        "console_scripts": [
            "onshape-to-robot=onshape_to_robot:onshape_to_robot.main",
            "onshape-to-robot-bullet=onshape_to_robot:bullet.main",
            "onshape-to-robot-clear-cache=onshape_to_robot:clear_cache.main",
            "onshape-to-robot-edit-shape=onshape_to_robot:edit_shape.main",
            "onshape-to-robot-pure-sketch=onshape_to_robot:pure_sketch.main",
        ]
    },
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
