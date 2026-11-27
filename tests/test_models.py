"""
Checks that the exported models are valid, not only identical to the expected
outputs (an expected output can be a broken model): MuJoCo loads and simulates
them, URDF and SDF are well-formed robots, and the three formats describe the
same joints and masses.
"""

from __future__ import annotations

import contextlib
import io
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from export_case import export, load_cases
from onshape_replay import replay

CASES = load_cases()
EXPECTED_DIR = Path(__file__).parent / "expected"
CASE_IDS = [case["name"] for case in CASES]


@pytest.fixture(scope="module")
def exports(tmp_path_factory):
    """Exports of the cases (with their assets), made once: exports[case][format]"""
    cache: dict = {}

    def get(case: dict, output_format: str) -> Path:
        extension = {"urdf": "urdf", "sdf": "sdf", "mujoco": "xml"}[output_format]
        if (EXPECTED_DIR / case["name"] / f"robot.{extension}.error").exists():
            pytest.skip(f"the {output_format} export of this case is expected to fail")
        key = (case["name"], output_format)
        if key not in cache:
            directory = (
                tmp_path_factory.mktemp(f"{case['name']}-{output_format}") / "robot"
            )
            with contextlib.redirect_stdout(io.StringIO()), replay(case["name"]):
                cache[key] = export(case, output_format, directory)
        return cache[key]

    return get


# ------------------------------------------------------------------- readers
def urdf_robot(path: Path) -> dict:
    root = ET.parse(path).getroot()
    links = [link.get("name") for link in root.findall("link")]
    joints = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        joints[joint.get("name")] = {
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "limits": (
                (float(limit.get("lower")), float(limit.get("upper")))
                if limit is not None and limit.get("lower") is not None
                else None
            ),
        }
    mass = sum(
        float(link.find("inertial/mass").get("value"))
        for link in root.findall("link")
        if link.find("inertial/mass") is not None
    )
    return {"links": links, "joints": joints, "mass": mass}


def sdf_robot(path: Path) -> dict:
    model = ET.parse(path).getroot().find("model")
    links = [link.get("name") for link in model.findall("link")]
    frames = [frame.get("name") for frame in model.findall("frame")]
    joints = {}
    for joint in model.findall("joint"):
        lower, upper = joint.find("axis/limit/lower"), joint.find("axis/limit/upper")
        joints[joint.get("name")] = {
            "type": joint.get("type"),
            "parent": joint.find("parent").text,
            "child": joint.find("child").text,
            "limits": (
                (float(lower.text), float(upper.text)) if lower is not None else None
            ),
        }
    relative_to = {
        el.get("relative_to") for el in model.iter() if el.get("relative_to")
    }
    mass = sum(float(link.find("inertial/mass").text) for link in model.findall("link"))
    return {
        "links": links,
        "frames": frames,
        "joints": joints,
        "relative_to": relative_to,
        "mass": mass,
    }


def mujoco_robot(path: Path) -> dict:
    root = ET.parse(path).getroot()
    joints = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        if name is None or name.endswith("_freejoint"):
            continue
        joint_range = joint.get("range")
        joints[name] = {
            "type": {"hinge": "revolute", "slide": "prismatic", "ball": "ball"}.get(
                joint.get("type"), "revolute"
            ),
            "limits": (
                tuple(float(x) for x in joint_range.split()) if joint_range else None
            ),
        }
    mass = sum(float(inertial.get("mass")) for inertial in root.iter("inertial"))
    return {"joints": joints, "mass": mass}


def single_tree(links: list, joints: dict, root_parent=None) -> list:
    """Problems in a set of links connected by joints (parent/child names)"""
    problems = []
    if len(set(links)) != len(links):
        problems.append("duplicated link names")
    children = []
    for name, joint in joints.items():
        if joint["parent"] == root_parent:
            continue
        if joint["parent"] not in links or joint["child"] not in links:
            problems.append(f"joint {name} references a missing link")
        children.append(joint["child"])
    if len(set(children)) != len(children):
        problems.append("a link has several parents")
    return problems


# ------------------------------------------------------------------- tests
@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_mujoco_loads_and_simulates(case, exports):
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(exports(case, "mujoco")))
    data = mujoco.MjData(model)
    for _ in range(50):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))


@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_urdf_structure(case, exports):
    robot = urdf_robot(exports(case, "urdf"))
    assert single_tree(robot["links"], robot["joints"]) == []
    children = {joint["child"] for joint in robot["joints"].values()}
    roots = [link for link in robot["links"] if link not in children]
    assert len(roots) == 1, f"URDF needs exactly one root link, got {roots}"


@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_sdf_structure(case, exports):
    robot = sdf_robot(exports(case, "sdf"))
    assert single_tree(robot["links"], robot["joints"], root_parent="world") == []
    known = set(robot["links"]) | set(robot["frames"]) | set(robot["joints"])
    assert robot["relative_to"] <= known, "relative_to references unknown names"


@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_formats_are_consistent(case, exports):
    urdf = urdf_robot(exports(case, "urdf"))
    sdf = sdf_robot(exports(case, "sdf"))
    mujoco = mujoco_robot(exports(case, "mujoco"))

    # Joints of the robot: URDF adds a fixed joint per frame, SDF a fixed
    # joint per link fixed to the world, MuJoCo doesn't write fixed joints
    sdf_joints = {
        name: joint
        for name, joint in sdf["joints"].items()
        if joint["parent"] != "world" and joint["type"] != "fixed"
    }
    urdf_joints = {
        name: joint
        for name, joint in urdf["joints"].items()
        if not name.endswith("_frame") and joint["type"] != "fixed"
    }
    assert set(sdf_joints) == set(mujoco["joints"])
    # URDF only exports the first base link (and what is attached to it)
    assert set(urdf_joints) <= set(sdf_joints)

    for name, joint in sdf_joints.items():
        expected_type = "revolute" if joint["type"] == "continuous" else joint["type"]
        assert mujoco["joints"][name]["type"] == expected_type, name
        if mujoco["joints"][name]["limits"] is not None:
            assert np.allclose(joint["limits"], mujoco["joints"][name]["limits"]), name
        if name in urdf_joints:
            assert urdf_joints[name]["type"] == joint["type"], name
            if joint["limits"] is not None:
                # URDF numbers are written with 6 significant digits
                assert np.allclose(
                    urdf_joints[name]["limits"], joint["limits"], atol=1e-4
                ), name

    assert sdf["mass"] == pytest.approx(mujoco["mass"], rel=1e-5)
