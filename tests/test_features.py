"""
Tests on the "features" document, built for the tests (see cases.json): the
behaviors are checked explicitly, in addition to the comparison with the
expected outputs.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from helpers import load_mujoco, run_cli

EXTENSIONS = {"urdf": "urdf", "sdf": "sdf", "mujoco": "xml"}


def export(tmp_path: Path, case: str, output_format: str, **entries) -> Path:
    directory = tmp_path / case
    run_cli(case, directory, {"output_format": output_format, **entries})
    return directory / f"robot.{EXTENSIONS[output_format]}"


def mujoco_joints(path: Path) -> dict:
    joints = {}
    for joint in ET.parse(path).getroot().iter("joint"):
        if joint.get("name") and not joint.get("name").endswith("_freejoint"):
            joints[joint.get("name")] = joint.attrib
    return joints


def urdf_joints(path: Path) -> dict:
    return {
        joint.get("name"): joint for joint in ET.parse(path).getroot().findall("joint")
    }


# ------------------------------------------------------------------- mechanisms
def test_continuous_wheel_ignores_onshape_limits(tmp_path):
    """dof_left_wheel has limits enabled in Onshape (+/- 30 deg)"""
    joints = mujoco_joints(export(tmp_path, "features-mechanisms", "mujoco"))
    assert joints["left_wheel"]["type"] == "hinge"
    assert "range" not in joints["left_wheel"]


def test_ball_joint(tmp_path):
    """dof_shoulder is a ball mate, its cone is limited by the #cone variable"""
    joints = mujoco_joints(export(tmp_path / "mujoco", "features-mechanisms", "mujoco"))
    assert joints["shoulder"]["type"] == "ball"
    assert np.allclose(
        [float(x) for x in joints["shoulder"]["range"].split()], [0, np.pi / 4]
    )

    model = ET.parse(export(tmp_path / "sdf", "features-mechanisms", "sdf")).getroot()
    shoulder = next(j for j in model.iter("joint") if j.get("name") == "shoulder")
    assert shoulder.get("type") == "ball"

    with pytest.raises(Exception, match="ball_to_euler"):
        export(tmp_path / "urdf", "features-mechanisms", "urdf")


def test_ball_to_euler(tmp_path):
    joints = urdf_joints(export(tmp_path, "features-mechanisms-euler", "urdf"))
    euler = {
        name: joints[name].get("type") for name in joints if name.startswith("shoulder")
    }
    assert euler == {f"shoulder_{axis}": "revolute" for axis in "xyz"}


def test_limits_from_variables(tmp_path):
    """
    dof_slide is limited by "#slide_min" (-1 cm) and "max(#slide_max, 2 cm)"
    (#slide_max = #slide_half * 2 = 2.54 cm); limits are relative to the
    position of the slider in the assembly (2.54 cm)
    """
    joints = mujoco_joints(export(tmp_path, "features-mechanisms", "mujoco"))
    limits = [float(x) for x in joints["slide"]["range"].split()]
    assert np.allclose(limits, [-0.01 - 0.0254, 0.0254 - 0.0254])


def test_gear_relations(tmp_path):
    """
    "Gear A" (ratio 2) and "Gear B (reversed)" (ratio 3, reversed direction).
    The signs were checked in Onshape: rotating gear_a1 by 0.1 rad rotates
    gear_a2 by -0.2 rad, and rotating gear_b1 by 0.1 rad rotates gear_b2 by
    0.3 rad.
    """
    urdf = urdf_joints(export(tmp_path / "urdf", "features-mechanisms-euler", "urdf"))
    mimic = {
        name: joint.find("mimic")
        for name, joint in urdf.items()
        if joint.find("mimic") is not None
    }
    assert {
        n: (m.get("joint"), float(m.get("multiplier"))) for n, m in mimic.items()
    } == {
        "gear_a2": ("gear_a1", -2.0),
        "gear_b2": ("gear_b1", 3.0),
    }

    text = export(tmp_path / "mujoco", "features-mechanisms", "mujoco").read_text()
    equalities = re.findall(
        r'<joint joint1="([^"]*)" joint2="([^"]*)" polycoef="0 ([^ ]*) ', text
    )
    assert {(a, b, float(r)) for a, b, r in equalities} == {
        ("gear_a2", "gear_a1", -2.0),
        ("gear_b2", "gear_b1", 3.0),
    }


def test_fixed_joint_and_merged_bodies(tmp_path):
    """
    dof_fixed_block is a fixed joint, while the parts attached with fix_spacer
    (a revolute mate named fix_), with the mate group and with an unnamed
    fastened mate (the surface) are merged into the base link
    """
    path = export(tmp_path, "features-mechanisms-euler", "urdf")
    assert urdf_joints(path)["fixed_block"].get("type") == "fixed"
    links = {link.get("name") for link in ET.parse(path).getroot().findall("link")}
    robot_links = {
        name for name in links if not name.startswith(("shoulder_", "frame"))
    }
    assert robot_links == {
        "base",
        "wheel",
        "arm",
        "block",
        "slider",
        "gear",
        "gear_2",
        "gear_3",
        "gear_4",
    }
    text = path.read_text()
    start = text.index('<link name="base">')
    base = text[start : text.index("</link>", start)]
    # Parts are named in the order of the export: the blocks merged into the
    # base come first
    assert set(re.findall(r"<!-- Part (\S+) -->", base)) == {
        "base",
        "block",
        "block_2",
        "sheet",
    }


def test_surface_without_collision(tmp_path, capsys):
    """The surface part (no volume) is exported as a visual only"""
    path = export(tmp_path, "features-mechanisms", "mujoco")
    text = path.read_text()
    assert re.search(r'class="visual"[^>]*mesh="sheet"', text)
    assert not re.search(r'class="collision"[^>]*mesh="sheet"', text)
    assert '<mesh file="sheet.stl" inertia="shell"' in text
    assert "is a surface" in capsys.readouterr().out
    load_mujoco(path)


# ------------------------------------------------------------------- closures
def test_closures(tmp_path, capsys):
    text = export(tmp_path, "features-closures", "mujoco").read_text()
    constraints = set(re.findall(r'<(weld|connect) site1="([^"]*)"', text))
    assert constraints == {
        ("weld", "closing_fastened_1"),
        ("connect", "closing_revolute_1"),
        ("connect", "closing_revolute_1_z"),
        ("connect", "closing_ball_1"),
    }
    assert "slider is not supported" in capsys.readouterr().out
    load_mujoco(tmp_path / "features-closures" / "robot.xml")


# ------------------------------------------------------------------- configurations
@pytest.mark.parametrize(
    "case, elbow, wrist",
    [
        # Default configuration: Size = "Small arm", Limited = true
        ("features-configured", (-10, 30), (-20, 20)),
        ("features-configured-large", (-45, 90), (-20, 60)),
        # The limits of the elbow are disabled by the configuration
        ("features-configured-unlimited", None, (-20, 20)),
    ],
)
def test_configured_limits(tmp_path, case, elbow, wrist):
    """
    The limits of dof_elbow (and their activation) and the upper limit of
    dof_wrist depend on the configuration (list parameter "Size", boolean
    parameter "Limited", whose ids differ from their names)
    """
    joints = mujoco_joints(export(tmp_path, case, "mujoco"))
    for name, expected in (("elbow", elbow), ("wrist", wrist)):
        if expected is None:
            assert "range" not in joints[name], name
        else:
            limits = [float(x) for x in joints[name]["range"].split()]
            assert np.allclose(limits, np.radians(expected)), name


def test_unknown_configuration_value(tmp_path):
    with pytest.raises(Exception, match="should be true or false"):
        export(
            tmp_path,
            "features-configured-large",
            "urdf",
            configuration={"Size": "Small arm", "Limited": "maybe"},
        )
