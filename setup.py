import setuptools

with open("README-pypi.md", "r", encoding="utf-8") as stream:
    long_description = stream.read()

setuptools.setup(
    name="onshape_to_robot",
    version="1.7.3",
    author="Rhoban team",
    author_email="team@rhoban.com",
    description="Converting Onshape assembly to robot definition (URDF, SDF, MuJoCo) through Onshape API ",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/rhoban/onshape-to-robot/",
    packages=setuptools.find_packages(),
    entry_points={
        "console_scripts": [
            "onshape-to-robot=onshape_to_robot:export.main",
            "onshape-to-robot-bullet=onshape_to_robot:bullet.main",
            "onshape-to-robot-mujoco=onshape_to_robot:mujoco.main",
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
    keywords="robot robotics cad design onshape bullet pybullet mujoco urdf sdf gazebo ros model kinematics",
    install_requires=[
        "numpy",
        "requests",
        "commentjson",
        "colorama>=0.4.6",
        "numpy-stl",
        "transforms3d",
        "python-dotenv",
    ],
    extras_requires={
        "pymeshlab": ["pymeshlab"],
    },
    include_package_data=True,
    package_data={"": ["bullet/*", "assets/*", "README*.md"]},
    python_requires=">=3.9",
)
