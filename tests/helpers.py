"""
Helpers shared by the tests: configurations, small robots built in Python,
and the onshape-to-robot command run on a recorded case.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from unittest import mock

import numpy as np

from export_case import load_cases
from onshape_replay import replay

from onshape_to_robot.config import Config
from onshape_to_robot.robot import Joint, Link, Part, Robot

ROOT = Path(__file__).parent.parent


def make_config(directory: Path, **entries) -> Config:
    directory.mkdir(parents=True, exist_ok=True)
    config = {"document_id": "a", "output_format": "urdf", **entries}
    (directory / "config.json").write_text(json.dumps(config))
    return Config(str(directory))


def two_links(joint_type: str, limits=None, name: str = "j") -> Robot:
    robot = Robot("r")
    a, b = Link("a"), Link("b")
    robot.links, robot.base_links = [a, b], [a]
    robot.joints = [Joint(name, joint_type, a, b, np.eye(4), limits=limits)]
    return robot


def part(name: str, T=None, mass=1.0, meshes=(), shapes=()) -> Part:
    return Part(
        name,
        np.eye(4) if T is None else T,
        mass,
        np.zeros(3),
        np.eye(3) * 1e-3,
        list(meshes),
        list(shapes),
    )


def translation(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def run_cli(case_name: str, directory: Path, entries: dict, *args: str):
    """
    Runs the onshape-to-robot command on a recorded case, in directory, with
    entries added to its config.json
    """
    from onshape_to_robot import export

    case = next(case for case in load_cases() if case["name"] == case_name)
    config = {
        "document_id": case["document_id"],
        "version_id": case["version_id"],
        "output_filename": "robot",
        **({"assembly_name": case["assembly"]} if case.get("assembly") else {}),
        **case.get("config", {}),
        **entries,
    }
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "config.json").write_text(json.dumps(config))
    argv = ["onshape-to-robot", str(directory), *args]
    with replay(case_name), mock.patch.object(sys, "argv", argv):
        export.main()


def load_mujoco(path: Path):
    """
    Checks that MuJoCo loads the model, if MuJoCo is installed (this is an
    additional check: the rest of the test is run anyway)
    """
    try:
        import mujoco
    except ImportError:
        return None
    return mujoco.MjModel.from_xml_path(str(path))


def python_env(**extra) -> dict:
    """Environment to run the code of this repository in a subprocess"""
    env = {**os.environ, "PYTHONPATH": str(ROOT), **extra}
    env.pop("ONSHAPE_API", None)
    return env


def write_stl(path: Path):
    from stl import mesh as stl_mesh

    path.parent.mkdir(parents=True, exist_ok=True)
    m = stl_mesh.Mesh(np.zeros(1, dtype=stl_mesh.Mesh.dtype))
    m.vectors[0] = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]
    m.save(str(path))
