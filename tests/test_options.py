"""
Tests of the config.json entries and of the command line options, on recorded
robots (see cases.json): each test checks the documented effect of an option.
"""

from __future__ import annotations

import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from helpers import load_mujoco, make_config, run_cli

from onshape_to_robot.robot import Joint, Link, Robot


def export(tmp_path: Path, case: str, output_format: str, **entries) -> Path:
    """Exports a recorded case with additional config.json entries"""
    directory = tmp_path / case
    run_cli(case, directory, {"output_format": output_format, **entries})
    return (
        directory
        / f"robot.{ {'urdf': 'urdf', 'sdf': 'sdf', 'mujoco': 'xml'}[output_format] }"
    )


def urdf(path: Path) -> ET.Element:
    return ET.parse(path).getroot()


def part_names(path: Path) -> list:
    return re.findall(r"<!-- Part (\S+) -->", path.read_text())


def link_masses(root: ET.Element) -> dict:
    return {
        link.get("name"): float(link.find("inertial/mass").get("value"))
        for link in root.findall("link")
        if link.find("inertial/mass") is not None
    }


# ------------------------------------------------------------------- parts
def test_ignore(tmp_path):
    reference = urdf(export(tmp_path / "ref", "quadruped", "urdf"))
    root = urdf(export(tmp_path, "quadruped", "urdf", ignore={"horn*": "all"}))
    text = ET.tostring(root, encoding="unicode")
    assert "horn" in ET.tostring(reference, encoding="unicode")
    assert "horn" not in text


def test_ignore_collision_only_with_exception(tmp_path):
    ignore = {"*": "collision", "!body": "collision"}
    root = urdf(export(tmp_path, "quadruped", "urdf", ignore=ignore))
    collisions = [
        mesh.get("filename")
        for mesh in root.iter("mesh")
        if mesh in [m for c in root.iter("collision") for m in c.iter("mesh")]
    ]
    visuals = [m.get("filename") for v in root.iter("visual") for m in v.iter("mesh")]
    assert collisions and all(f.endswith("/body.stl") for f in collisions)
    assert len(visuals) > len(collisions)


def test_color(tmp_path):
    root = urdf(export(tmp_path, "quadruped", "urdf", color=[1, 0, 0, 1]))
    colors = {color.get("rgba") for color in root.iter("color")}
    assert colors == {"1 0 0 1"}


def test_no_dynamics(tmp_path):
    root = urdf(export(tmp_path, "quadruped", "urdf", no_dynamics=True))
    assert set(link_masses(root).values()) == {0.0}


def test_include_configuration_suffix(tmp_path):
    with_suffix = part_names(export(tmp_path / "a", "spm", "urdf"))
    without = part_names(
        export(tmp_path / "b", "spm", "urdf", include_configuration_suffix=False)
    )
    assert len(with_suffix) == len(without)
    # Configured parts (e.g. library screws, with a long configuration hashed
    # in their name) lose their configuration suffix
    assert any(re.search(r"_[0-9a-f]{32}", name) for name in with_suffix)
    assert not any(re.search(r"_[0-9a-f]{32}", name) for name in without)


def test_draw_frames(tmp_path):
    """
    The part placing a frame_ (here the marker of planar-2dof) is exported
    when draw_frames is enabled, and the frame is still exported
    """
    without = export(tmp_path / "a", "planar-2dof", "urdf")
    with_frames = export(tmp_path / "b", "planar-2dof", "urdf", draw_frames=True)
    added = set(part_names(with_frames)) - set(part_names(without))
    assert added == {"frame"}

    def frames(path):
        return {
            j.get("name")
            for j in urdf(path).findall("joint")
            if j.get("name").endswith("_frame")
        }

    assert frames(with_frames) == frames(without) != set()


# ------------------------------------------------------------------- joints
def test_ignore_limits(tmp_path):
    root = urdf(export(tmp_path, "megabot", "urdf", ignore_limits=True))
    limits = {
        (joint.find("limit").get("lower"), joint.find("limit").get("upper"))
        for joint in root.findall("joint")
        if joint.get("type") == "revolute"
    }
    # Only the default limits (+/- pi, rounded to round_decimals)
    assert limits == {("-3.14159265359", "3.14159265359")}


def test_joint_properties_urdf(tmp_path):
    properties = {
        "*": {"max_effort": 20, "max_velocity": 6},
        "leg1_a": {"limits": [-0.5, 0.5], "friction": 0.3},
    }
    root = urdf(export(tmp_path, "quadruped", "urdf", joint_properties=properties))
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    limit = joints["leg1_a"].find("limit")
    assert (limit.get("effort"), limit.get("velocity")) == ("20", "6")
    assert (limit.get("lower"), limit.get("upper")) == ("-0.5", "0.5")
    assert joints["leg1_a"].find("joint_properties").get("friction") == "0.3"
    assert joints["leg2_a"].find("limit").get("effort") == "20"


def test_joint_properties_mujoco(tmp_path):
    properties = {
        "leg1_*": {"type": "velocity", "kv": 5, "forcerange": 2},
        "leg2_a": {"actuated": False},
        "leg3_a": {"damping": 0.5, "armature": 0.01},
    }
    path = export(tmp_path, "quadruped", "mujoco", joint_properties=properties)
    text = path.read_text()
    assert len(re.findall(r'<velocity [^>]*joint="leg1_', text)) == 3
    assert 'forcerange="-2 2"' in text
    assert 'joint="leg2_a"' not in text
    assert re.search(r'<joint [^>]*name="leg3_a"[^>]*damping="0.5"', text)
    load_mujoco(path)


def test_ball_to_euler(tmp_path):
    from onshape_to_robot.exporter_urdf import ExporterURDF
    from onshape_to_robot.processor_ball_to_euler import ProcessorBallToEuler

    config = make_config(tmp_path, ball_to_euler=True, ball_to_euler_order="zyx")
    robot = Robot("r")
    a, b = Link("a"), Link("b")
    robot.links, robot.base_links = [a, b], [a]
    robot.joints = [Joint("hip", Joint.BALL, a, b, np.eye(4))]
    ProcessorBallToEuler(config).process(robot)
    assert [joint.name for joint in robot.joints] == ["hip_z", "hip_y", "hip_x"]
    assert {joint.joint_type for joint in robot.joints} == {Joint.REVOLUTE}
    # The URDF export now works
    ET.fromstring(ExporterURDF(config).build(robot))


def joint_names(path: Path) -> set:
    return {
        joint.get("name")
        for joint in urdf(path).findall("joint")
        if not joint.get("name").endswith("_frame")
    }


def test_rename_dofs(tmp_path):
    path = export(
        tmp_path, "quadruped", "urdf", rename_dofs={"leg1_a": "front_left_hip"}
    )
    names = joint_names(path)
    assert "front_left_hip" in names and "leg1_a" not in names
    assert "leg2_a" in names


def test_rename_dofs_regular_expression(tmp_path):
    rename = {r"leg(\d)_(.)": r"l\1_\2"}
    names = joint_names(export(tmp_path, "quadruped", "urdf", rename_dofs=rename))
    assert names == {f"l{leg}_{dof}" for leg in range(1, 5) for dof in "abc"}


def test_rename_dofs_first_matching_key_applies(tmp_path):
    rename = {"leg1_a": "first", r"leg1_(.)": r"second_\1"}
    names = joint_names(export(tmp_path, "quadruped", "urdf", rename_dofs=rename))
    assert "first" in names and "second_a" not in names
    assert {"second_b", "second_c"} <= names


def test_rename_dofs_used_by_joint_properties(tmp_path):
    path = export(
        tmp_path,
        "quadruped",
        "urdf",
        rename_dofs={"leg1_a": "front_left_hip"},
        joint_properties={"front_left_hip": {"max_effort": 7}},
    )
    joint = next(
        j for j in urdf(path).findall("joint") if j.get("name") == "front_left_hip"
    )
    assert joint.find("limit").get("effort") == "7"


def test_rename_dofs_unused_entry_warns(tmp_path, capsys):
    export(tmp_path, "quadruped", "urdf", rename_dofs={"no_such_dof": "x"})
    assert "matched no DOF" in capsys.readouterr().out


@pytest.mark.parametrize(
    "rename, error",
    [
        ({"leg(": "x"}, "invalid rename_dofs pattern"),
        ({"leg1_a": "same", "leg2_a": "same"}, "several DOFs are named same"),
        (["leg1_a"], "should be a dict"),
    ],
)
def test_rename_dofs_errors(tmp_path, rename, error):
    with pytest.raises(Exception, match=error):
        export(tmp_path, "quadruped", "urdf", rename_dofs=rename)


def test_joint_properties_sdf(tmp_path):
    properties = {"leg1_a": {"max_effort": 20, "max_velocity": 6, "friction": 0.3}}
    path = export(tmp_path, "quadruped", "sdf", joint_properties=properties)
    model = ET.parse(path).getroot().find("model")
    joint = next(j for j in model.findall("joint") if j.get("name") == "leg1_a")
    assert joint.find("axis/limit/effort").text == "20"
    assert joint.find("axis/limit/velocity").text == "6"
    assert joint.find("axis/dynamics/friction").text == "0.3"


def test_mujoco_actuator_ctrlrange(tmp_path):
    """A velocity actuator with explicit limits gets them as control range"""
    properties = {"leg1_a": {"type": "velocity", "limits": [-2, 2]}}
    text = export(
        tmp_path, "quadruped", "mujoco", joint_properties=properties
    ).read_text()
    actuator = re.search(r'<velocity [^>]*joint="leg1_a"[^>]*>', text).group(0)
    assert 'ctrlrange="-2 2"' in actuator
    position = re.search(r'<position [^>]*joint="leg2_a"[^>]*>', text).group(0)
    assert "ctrlrange" not in position


# ------------------------------------------------------------------- links
def test_use_fixed_links(tmp_path):
    root = urdf(export(tmp_path, "quadruped", "urdf", use_fixed_links=True))
    fixed = [j for j in root.findall("joint") if j.get("name").endswith("_fixed")]
    # One link (and fixed joint) per part
    assert len(fixed) == len(part_names(tmp_path / "quadruped" / "robot.urdf"))
    for link in root.findall("link"):
        assert len(link.findall("visual")) <= 1


def test_add_dummy_base_link(tmp_path):
    root = urdf(export(tmp_path, "quadruped", "urdf", add_dummy_base_link=True))
    children = {joint.find("child").get("link") for joint in root.findall("joint")}
    roots = [
        link.get("name")
        for link in root.findall("link")
        if link.get("name") not in children
    ]
    assert roots == ["base_link"]


@pytest.mark.parametrize("merge_stls", [True, "visual", "collision"])
def test_merge_stls(tmp_path, merge_stls):
    path = export(tmp_path, "quadruped", "mujoco", merge_stls=merge_stls)
    text = path.read_text()
    merged = re.findall(r'<mesh file="(merged/[^"]*)"', text)
    assert merged
    assert all((tmp_path / "quadruped" / "assets" / f).exists() for f in merged)
    if merge_stls is True:
        # A single visual and a single collision mesh per link
        bodies = re.findall(r"<body name=.*?(?=<body |</body>)", text, re.S)
        for body in bodies:
            assert len(re.findall(r'class="visual"', body)) <= 1
    load_mujoco(path)


def test_no_collision_meshes(tmp_path):
    root = urdf(export(tmp_path, "quadruped", "urdf", no_collision_meshes=True))
    assert root.find(".//collision") is None
    assert root.find(".//visual") is not None


def test_collisions_as_visual(tmp_path):
    ignore = {"*": "collision", "!body": "collision"}
    root = urdf(
        export(tmp_path, "quadruped", "urdf", ignore=ignore, collisions_as_visual=True)
    )
    visuals = [m.get("filename") for v in root.iter("visual") for m in v.iter("mesh")]
    collisions = [
        m.get("filename") for c in root.iter("collision") for m in c.iter("mesh")
    ]
    assert sorted(visuals) == sorted(collisions)


def test_set_zero_mass_to_fixed(tmp_path):
    """The base of irb6620 is fixed in the assembly"""
    root = urdf(export(tmp_path, "irb6620", "urdf", set_zero_mass_to_fixed=True))
    children = {joint.find("child").get("link") for joint in root.findall("joint")}
    base = next(
        link for link in root.findall("link") if link.get("name") not in children
    )
    assert float(base.find("inertial/mass").get("value")) == 0
    assert sum(link_masses(root).values()) > 0


# ------------------------------------------------------------------- exporters
def test_package_name(tmp_path):
    root = urdf(export(tmp_path, "quadruped", "urdf", package_name="my_robot"))
    files = {mesh.get("filename") for mesh in root.iter("mesh")}
    assert files and all(f.startswith("package://my_robot/assets/") for f in files)


@pytest.mark.parametrize("output_format", ["urdf", "sdf", "mujoco"])
def test_additional_xml(tmp_path, output_format):
    directory = tmp_path / "quadruped"
    directory.mkdir(parents=True)
    (directory / "extra.xml").write_text('<custom_tag value="42"/>')
    path = export(tmp_path, "quadruped", output_format, additional_xml="extra.xml")
    assert '<custom_tag value="42"/>' in path.read_text()


def test_equalities(tmp_path):
    """spm has closing_ mates, exported as MuJoCo equality constraints"""
    equalities = {"closing_*": {"solref": "0.002 1"}}
    path = export(tmp_path, "spm", "mujoco", equalities=equalities)
    constraints = re.findall(r"<(?:weld|connect) [^>]*>", path.read_text())
    assert constraints and all('solref="0.002 1"' in c for c in constraints)
    load_mujoco(path)


def test_robot_name_and_output_filename(tmp_path):
    directory = tmp_path / "quadruped"
    entries = {
        "output_format": "urdf",
        "robot_name": "my_quad",
        "output_filename": "quad",
    }
    run_cli("quadruped", directory, entries)
    root = urdf(directory / "quad.urdf")
    assert root.get("name") == "my_quad"


def test_round_decimals(tmp_path):
    text = export(tmp_path, "quadruped", "urdf", round_decimals=3).read_text()
    xyz = re.findall(r'<origin xyz="([^"]*)"', text)
    numbers = [n for value in xyz for n in value.split()]
    assert numbers and all(len(n.split(".")[1]) <= 3 for n in numbers if "." in n)


SCAD = """
cube([10, 20, 30], center=true);
translate([0, 0, 20]) cylinder(r=5, h=10, center=true);
sphere(3);
"""


def scad_shapes(path: Path, output_format: str, node: str) -> dict:
    """Shapes of the given node (visual or collision): {kind: [sizes]}"""
    root = ET.parse(path).getroot()
    shapes = {"box": [], "cylinder": [], "sphere": []}
    for element in root.iter(node):
        for kind in shapes:
            for shape in element.iter(kind):
                if output_format == "urdf":
                    attributes = {
                        "box": ["size"],
                        "cylinder": ["length", "radius"],
                        "sphere": ["radius"],
                    }
                    shapes[kind].append(
                        " ".join(shape.get(a) for a in attributes[kind])
                    )
                else:
                    shapes[kind].append(" ".join(child.text for child in shape))
    return shapes


@pytest.mark.parametrize("output_format", ["urdf", "sdf"])
@pytest.mark.parametrize("as_visual", [False, True])
def test_use_scads_pure_shapes(tmp_path, output_format, as_visual):
    """
    Pure shapes from a .scad file, in URDF and SDF (MuJoCo is tested elsewhere),
    also as visuals with collisions_as_visual
    """
    if shutil.which("openscad") is None:
        pytest.skip("OpenSCAD not installed")
    directory = tmp_path / "quadruped"
    (directory / "assets").mkdir(parents=True)
    (directory / "assets" / "body.scad").write_text(SCAD)
    path = export(
        tmp_path,
        "quadruped",
        output_format,
        use_scads=True,
        collisions_as_visual=as_visual,
    )
    # body.stl is used by two parts, sizes are in meters
    expected = {
        "box": ["0.01 0.02 0.03"] * 2,
        "cylinder": ["0.01 0.005"] * 2,
        "sphere": ["0.003"] * 2,
    }
    assert scad_shapes(path, output_format, "collision") == expected
    visual = scad_shapes(path, output_format, "visual")
    assert visual == (expected if as_visual else {k: [] for k in expected})


def test_several_assemblies_without_assembly_name(tmp_path):
    """The document of subasm-robot contains several assemblies"""
    with pytest.raises(Exception, match="Multiple assemblies found"):
        export(tmp_path, "subasm-robot", "urdf", assembly_name=None)


def test_unknown_assembly_name(tmp_path):
    with pytest.raises(Exception, match="Unable to find required assembly"):
        export(tmp_path, "subasm-robot", "urdf", assembly_name="no such assembly")


# ------------------------------------------------------------------- command line
def test_retrieve_then_convert_gives_the_same_export(tmp_path):
    direct = export(tmp_path / "direct", "quadruped", "urdf").read_text()
    directory = tmp_path / "steps" / "quadruped"
    run_cli("quadruped", directory, {"output_format": "urdf"}, "--retrieve")
    assert (directory / "robot.pkl").exists() and not (
        directory / "robot.urdf"
    ).exists()
    run_cli("quadruped", directory, {"output_format": "urdf"}, "--convert")
    converted = (directory / "robot.urdf").read_text()
    # Same export, except the robot name (the directory name)
    assert converted.replace('name="quadruped"', "") == direct.replace(
        'name="quadruped"', ""
    )


def test_convert_to_another_format(tmp_path):
    directory = tmp_path / "quadruped"
    run_cli("quadruped", directory, {"output_format": "urdf"}, "--retrieve")
    run_cli("quadruped", directory, {"output_format": "mujoco"}, "--convert")
    load_mujoco(directory / "robot.xml")


def test_safe_mode(tmp_path):
    """--safe disables post_import_commands and custom processors"""
    marker = tmp_path / "command_was_run"
    entries = {
        "output_format": "urdf",
        "post_import_commands": [f'touch "{marker}"'],
        "processors": ["ProcessorScad", "my_module:DoesNotExist"],
    }
    run_cli("quadruped", tmp_path / "safe" / "quadruped", entries, "--safe")
    assert not marker.exists()

    entries["processors"] = ["ProcessorScad"]
    run_cli("quadruped", tmp_path / "unsafe" / "quadruped", entries)
    assert marker.exists()
