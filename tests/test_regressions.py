"""
Regression tests for fixed bugs, each test reproducing the situation of one
bug. Most of them build a robot in Python or export a recorded robot (no
network access, no API key).

Tests that need an external tool (MuJoCo, OpenSCAD, CoACD, Gazebo) are skipped
when it is not installed.
"""

from __future__ import annotations

import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import xml.dom.minidom
from pathlib import Path
from unittest import mock

import numpy as np
import pytest

from helpers import (
    load_mujoco,
    make_config,
    part,
    python_env,
    run_cli,
    translation,
    two_links,
    write_stl,
)

from onshape_to_robot.geometry import Box, Mesh
from onshape_to_robot.robot import Joint, Link, Relation, Robot
from onshape_to_robot.exporter_mujoco import ExporterMuJoCo
from onshape_to_robot.exporter_sdf import ExporterSDF
from onshape_to_robot.exporter_urdf import ExporterURDF

EXPORTERS = [ExporterURDF, ExporterSDF, ExporterMuJoCo]


# ------------------------------------------------------------------- helpers
def merge(tmp_path: Path, parts: list, **entries) -> str:
    """Runs ProcessorMergeParts on a link made of parts, returns the MuJoCo XML"""
    from onshape_to_robot.processor_merge_parts import ProcessorMergeParts

    config = make_config(tmp_path / "robot", output_format="mujoco", **entries)
    write_stl(tmp_path / "robot" / "assets" / "p.stl")
    link = Link("l")
    link.parts = parts
    robot = Robot("r")
    robot.links, robot.base_links = [link], [link]
    ProcessorMergeParts(config).process(robot)
    return ExporterMuJoCo(config).build(robot)


# ------------------------------------------------------------------- exporters
@pytest.mark.parametrize("joint_type", ["continuous", "fixed", "ball"])
def test_sdf_joints_without_limits(tmp_path, joint_type):
    xml_text = ExporterSDF(make_config(tmp_path)).build(two_links(joint_type))
    joint = re.search(r'<joint name="j".*?</joint>', xml_text, re.S).group(0)
    assert "<lower>" not in joint
    if joint_type != "continuous":
        assert "<axis>" not in joint


def test_urdf_ball_joint_fails(tmp_path):
    with pytest.raises(Exception, match="ball_to_euler"):
        ExporterURDF(make_config(tmp_path)).build(two_links("ball"))


def test_mujoco_continuous_joint_is_hinge(tmp_path, capsys):
    xml_text = ExporterMuJoCo(make_config(tmp_path)).build(
        two_links("continuous", name="wheel")
    )
    joint = re.search(r'<joint [^>]*name="wheel"[^>]*>', xml_text).group(0)
    assert 'type="hinge"' in joint and "range=" not in joint
    assert "Unknown joint type" not in capsys.readouterr().out


def test_failed_export_keeps_previous_file(tmp_path):
    output = tmp_path / "robot.urdf"
    output.write_text("<robot>previous</robot>")
    with pytest.raises(Exception):
        ExporterURDF(make_config(tmp_path / "robot")).write_xml(
            two_links("ball"), str(output)
        )
    assert output.read_text() == "<robot>previous</robot>"


@pytest.mark.parametrize("exporter", EXPORTERS)
def test_names_are_escaped(tmp_path, exporter):
    config = make_config(
        tmp_path / "robot",
        robot_name='R&D "bot"',
        additional_xml="extra--x.xml",
        url="https://cad.onshape.com/documents/a/w/b--b/e/c",
    )
    (tmp_path / "robot" / "extra--x.xml").write_text("<!-- extra -->")
    robot = Robot(config.robot_name)
    a, b = Link("base & co"), Link("arm--1")
    robot.links, robot.base_links = [a, b], [a]
    joint = Joint('shoulder "L" & co', "revolute", a, b, np.eye(4), limits=(-1, 1))
    joint.relation = Relation("elbow<1> & co", 2.0)
    robot.joints = [joint]
    b.frames['tip & "end"---x'] = np.eye(4)
    b.parts = [part("p&q", meshes=[Mesh("assets/p.stl", np.ones(4))])]
    # Raises if the XML is not well-formed
    xml.dom.minidom.parseString(exporter(config).build(robot))


@pytest.mark.parametrize("exporter", EXPORTERS)
def test_numbers_are_rounded(tmp_path, exporter):
    robot = two_links("revolute", limits=(-np.pi, np.pi))
    robot.joints[0].relation = Relation("other", 1 / 3)
    xml_text = exporter(make_config(tmp_path)).build(robot)
    assert "3.141592653589793" not in xml_text
    assert "0.3333333333333333" not in xml_text


def test_mujoco_meshes_in_deterministic_order(tmp_path):
    outputs = []
    for seed in ("1", "2", "3"):
        directory = tmp_path / seed / "quadruped"
        code = (
            "import sys; sys.path.insert(0, sys.argv[1]);"
            "from helpers import run_cli; from pathlib import Path;"
            "run_cli('quadruped', Path(sys.argv[2]), {'output_format': 'mujoco'})"
        )
        subprocess.run(
            [sys.executable, "-c", code, str(Path(__file__).parent), str(directory)],
            check=True,
            capture_output=True,
            env=python_env(PYTHONHASHSEED=seed),
        )
        text = (directory / "robot.xml").read_text()
        outputs.append(re.findall(r'<mesh file="[^"]*"', text))
    assert outputs[0] == outputs[1] == outputs[2]


def test_sdf_mesh_uri_uses_model_directory(tmp_path):
    directory = tmp_path / "quad_dir"
    run_cli("quadruped", directory, {"output_format": "sdf", "robot_name": "quad"})
    text = (directory / "robot.sdf").read_text()
    assert "model://quad_dir/assets/" in text
    assert "model://quad/" not in text
    assert '<model name="quad"' in text


def test_sdf_loads_in_gazebo(tmp_path):
    if shutil.which("gz") is None:
        # The Gazebo job of the workflow sets O2R_REQUIRE_GAZEBO, so that a
        # broken Gazebo installation makes the test fail instead of skipping it
        if os.environ.get("O2R_REQUIRE_GAZEBO"):
            pytest.fail("Gazebo (gz) not found, but O2R_REQUIRE_GAZEBO is set")
        pytest.skip("Gazebo (gz) not installed")
    models = tmp_path / "models"
    run_cli(
        "quadruped", models / "quad_dir", {"output_format": "sdf", "robot_name": "quad"}
    )
    world = tmp_path / "world.sdf"
    world.write_text(
        '<?xml version="1.0" ?><sdf version="1.7"><world name="test">'
        "<include><uri>model://quad_dir</uri></include></world></sdf>"
    )
    result = subprocess.run(
        ["gz", "sim", "-s", "-r", "--iterations", "10", "-v", "3", str(world)],
        capture_output=True,
        text=True,
        timeout=120,
        env={**os.environ, "GZ_SIM_RESOURCE_PATH": str(models)},
    )
    log = result.stdout + result.stderr
    assert "Failed to load a world" not in log
    assert "could not be resolved" not in log


# ------------------------------------------------------------------- config
def test_url_with_query_string(tmp_path):
    config = make_config(
        tmp_path,
        url="https://cad.onshape.com/documents/aaa/v/bbb/e/ccc?renderMode=0",
    )
    assert (config.document_id, config.version_id, config.element_id) == (
        "aaa",
        "bbb",
        "ccc",
    )


def test_microversion_url_is_rejected(tmp_path):
    with pytest.raises(Exception, match="Microversion"):
        make_config(tmp_path, url="https://cad.onshape.com/documents/aaa/m/bbb/e/ccc")


def test_default_robot_name_on_windows():
    import ntpath
    import onshape_to_robot.config as config_module

    content = io.StringIO('{"document_id": "a", "output_format": "urdf"}')
    with (
        mock.patch.object(config_module.os, "path", ntpath),
        mock.patch.object(ntpath, "isdir", lambda path: False),
        mock.patch.object(ntpath, "exists", lambda path: True),
        mock.patch("builtins.open", lambda *args, **kwargs: content),
        mock.patch.object(config_module.os, "makedirs", lambda *args, **kwargs: None),
    ):
        config = config_module.Config(r"C:\Users\me\robots\my_robot\config.json")
    assert config.robot_name == "my_robot"


def test_custom_processor_imported_from_current_directory(tmp_path):
    (tmp_path / "my_project").mkdir()
    (tmp_path / "my_project" / "__init__.py").write_text("")
    (tmp_path / "my_project" / "processor.py").write_text(
        "from onshape_to_robot.processor import Processor\n"
        "class MyProcessor(Processor):\n    pass\n"
    )
    (tmp_path / "robot").mkdir()
    (tmp_path / "robot" / "config.json").write_text(
        json.dumps(
            {
                "document_id": "x",
                "version_id": "y",
                "output_format": "urdf",
                "processors": ["my_project.processor:MyProcessor"],
            }
        )
    )
    # Like the onshape-to-robot command, the current directory is not in the path
    script = tmp_path / "bin" / "onshape_to_robot_command.py"
    script.parent.mkdir()
    script.write_text("from onshape_to_robot.export import main\nmain()\n")
    result = subprocess.run(
        [sys.executable, str(script), "robot"],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        env=python_env(),
    )
    output = result.stdout + result.stderr
    assert "No module named" not in output
    # The export then stops on the missing API keys (no API call is made)
    assert "No Onshape API access key" in output


# ------------------------------------------------------------------- assembly
def test_expressions_max_min():
    from onshape_to_robot.expression import ExpressionParser

    parser = ExpressionParser()
    assert parser.eval_expr("1 + max(2, 3)") == pytest.approx(4)
    assert parser.eval_expr("2 * min(1, 4) mm") == pytest.approx(0.002)
    assert parser.eval_expr("max(1 m, 20 cm)") == pytest.approx(1)
    assert parser.eval_expr("15 deg") == pytest.approx(np.pi / 12)


def test_configured_limit_not_found():
    from onshape_to_robot.assembly import Assembly
    from onshape_to_robot.expression import ExpressionParser

    assembly = Assembly.__new__(Assembly)
    value = {
        "typeName": "BTMConfiguredValueByEnum",
        "message": {
            "enumValue": "Small",
            "value": {"message": {"expression": "1 deg"}},
        },
    }
    parameter = {
        "typeName": "BTMParameterConfigured",
        "message": {"configurationParameterId": "size", "values": [value]},
    }
    with pytest.raises(Exception, match="Could not find the value of dof_x"):
        assembly.read_parameter_value(
            parameter, "dof_x", ExpressionParser(), {"size": "Large"}
        )


def test_continuous_joint_ignores_limits(tmp_path):
    """dof_l1_r1 of megabot has limits enabled, renamed as a wheel"""
    from onshape_to_robot.onshape_api.client import Client

    def renaming(method):
        def wrapper(*args, **kwargs):
            text = json.dumps(method(*args, **kwargs))
            return json.loads(text.replace('dof_l1_r1"', 'dof_l1_r1_wheel"'))

        return wrapper

    with contextlib.ExitStack() as stack:
        for name in ("get_assembly", "get_features", "matevalues"):
            stack.enter_context(
                mock.patch.object(Client, name, renaming(getattr(Client, name)))
            )
        run_cli("megabot", tmp_path / "megabot", {"output_format": "mujoco"})

    text = (tmp_path / "megabot" / "robot.xml").read_text()
    joint = re.search(r'<joint [^>]*name="l1_r1_wheel"[^>]*>', text).group(0)
    assert 'type="hinge"' in joint and "range=" not in joint
    load_mujoco(tmp_path / "megabot" / "robot.xml")


# ------------------------------------------------------------------- processors
def test_ball_to_euler_patterns(tmp_path):
    from onshape_to_robot.processor_ball_to_euler import ProcessorBallToEuler

    processor = ProcessorBallToEuler(
        make_config(tmp_path, ball_to_euler=["shoulder_*"])
    )
    assert processor.should_replace(
        Joint("shoulder_left", "ball", None, None, np.eye(4))
    )
    assert not processor.should_replace(Joint("elbow", "ball", None, None, np.eye(4)))


def test_merge_stls_color_massless_parts(tmp_path):
    red = np.array([1, 0, 0, 1.0])
    xml_text = merge(
        tmp_path,
        [part("p", mass=0, meshes=[Mesh("assets/p.stl", red)])],
        merge_stls=True,
        no_dynamics=True,
    )
    assert re.search(r'<material [^>]*rgba="1 0 0 1"', xml_text)


def test_merge_stls_color_ignores_parts_without_mesh(tmp_path):
    red = np.array([1, 0, 0, 1.0])
    parts = [part("p", mass=1, meshes=[Mesh("assets/p.stl", red)]), part("q", mass=9)]
    xml_text = merge(tmp_path, parts, merge_stls=True)
    assert re.search(r'<material [^>]*rgba="1 0 0 1"', xml_text)


@pytest.mark.parametrize("merge_stls", [True, "visual", "collision"])
def test_merge_stls_shapes_not_duplicated(tmp_path, merge_stls):
    box = Box(np.eye(4), np.ones(3) * 0.1, np.ones(4), visual=False, collision=True)
    mesh = Mesh("assets/p.stl", np.ones(4), collision=False)
    other_mesh = Mesh("assets/p.stl", np.ones(4), collision=False)
    parts = [
        part("p", translation(1, 0, 0), meshes=[mesh], shapes=[box]),
        part("q", meshes=[other_mesh]),
    ]
    xml_text = merge(tmp_path, parts, merge_stls=merge_stls)
    boxes = re.findall(
        r'<geom class="collision" pos="([^"]*)"[^>]*type="box"', xml_text
    )
    assert boxes == ["1 0 0"]


def test_dummy_base_link_with_existing_base_link(tmp_path):
    from onshape_to_robot.processor_dummy_base_link import ProcessorDummyBaseLink

    robot = Robot("r")
    robot.links = [Link("base_link")]
    robot.base_links = list(robot.links)
    ProcessorDummyBaseLink(make_config(tmp_path, add_dummy_base_link=True)).process(
        robot
    )
    assert [link.name for link in robot.links] == ["base_link", "dummy_base_link"]


@pytest.mark.parametrize("path_name", ["robot", "robot with space"])
def test_use_scads(tmp_path, path_name):
    """body.stl has a .scad file, the other parts keep their collision mesh"""
    if shutil.which("openscad") is None:
        pytest.skip("OpenSCAD not installed")
    directory = tmp_path / path_name / "quadruped"
    (directory / "assets").mkdir(parents=True)
    (directory / "assets" / "body.scad").write_text(
        "cube([10, 20, 30], center=true);\n"
    )
    run_cli("quadruped", directory, {"output_format": "mujoco", "use_scads": True})
    text = (directory / "robot.xml").read_text()
    assert len(re.findall(r'type="mesh" class="collision"', text)) == 44
    boxes = re.findall(r'<geom class="collision"[^>]*type="box" size="([^"]*)"', text)
    assert boxes == ["0.005 0.01 0.015"] * 2
    assert not (directory / "_tmp_data.csg").exists()


def test_convex_decomposition(tmp_path, monkeypatch):
    pytest.importorskip("coacd")
    pytest.importorskip("trimesh")
    # CoACD results are cached in the home directory
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    directory = tmp_path / "quadruped"
    # Only the body parts, to keep the decomposition short
    entries = {
        "output_format": "mujoco",
        "convex_decomposition": True,
        "ignore": {"*": "collision", "!body": "collision"},
    }
    run_cli("quadruped", directory, entries)
    text = (directory / "robot.xml").read_text()
    files = re.findall(r'<mesh file="([^"]*convex_decomposition[^"]*)"', text)
    assert files and all(not os.path.isabs(f) for f in files)
    load_mujoco(directory / "robot.xml")


# ------------------------------------------------------------------- command line / API
def test_api_error_exit_code():
    from onshape_to_robot.onshape_api.onshape import Onshape

    onshape = Onshape.__new__(Onshape)
    onshape._url, onshape._logging = "https://example.com", False
    onshape._make_headers = lambda *args, **kwargs: {}
    response = mock.Mock(status_code=404, text="not found", headers={})
    with mock.patch("requests.request", return_value=response):
        with (
            pytest.raises(SystemExit) as error,
            contextlib.redirect_stdout(io.StringIO()),
        ):
            onshape.request("get", "/api/x")
    assert error.value.code not in (None, 0)


def test_credentials_in_config_json(tmp_path, monkeypatch):
    from onshape_to_robot.onshape_api.onshape import Onshape

    for name in (
        "ONSHAPE_API",
        "ONSHAPE_ACCESS_KEY",
        "ONSHAPE_SECRET_KEY",
        "ONSHAPE_SECRET_BEARER",
    ):
        monkeypatch.delenv(name, raising=False)
    credentials = tmp_path / "config.json"
    credentials.write_text(
        json.dumps(
            {
                "onshape_api": "https://cad.onshape.com",
                "onshape_access_key": "AK",
                "onshape_secret_key": "SK",
            }
        )
    )
    onshape = Onshape("https://cad.onshape.com", creds=str(credentials), logging=False)
    assert onshape._make_headers("get", "/api/x")["Authorization"].startswith("On AK:")


def test_convert_uses_current_properties(tmp_path):
    """Issue #206"""
    directory = tmp_path / "quadruped"
    run_cli("quadruped", directory, {"output_format": "mujoco"}, "--retrieve")
    entries = {
        "output_format": "mujoco",
        "robot_name": "renamed",
        "joint_properties": {"*_c": {"type": "velocity", "range": False, "kv": 5}},
        "geom_properties": {"horn*": {"collision": {"friction": "0.8"}}},
    }
    run_cli("quadruped", directory, entries, "--convert")
    text = (directory / "robot.xml").read_text()
    assert text.count("<velocity ") == 4
    assert text.count('friction="0.8"') == 12
    assert 'model="renamed"' in text

    # Removing them from config.json removes them from the export
    run_cli("quadruped", directory, {"output_format": "mujoco"}, "--convert")
    text = (directory / "robot.xml").read_text()
    assert "<velocity " not in text and 'friction="0.8"' not in text


def test_convert_with_save_pickle(tmp_path):
    directory = tmp_path / "quadruped"
    run_cli("quadruped", directory, {"output_format": "urdf"}, "--retrieve")
    run_cli(
        "quadruped", directory, {"output_format": "urdf"}, "--convert", "--save-pickle"
    )
    assert (directory / "robot.urdf").exists()


def fake_openscad(tmp_path: Path) -> str:
    """A fake openscad command printing the directory it is run in"""
    directory = tmp_path / "fake_bin"
    directory.mkdir()
    command = directory / "openscad"
    command.write_text('#!/bin/sh\necho "RUN_IN=$PWD"\n')
    command.chmod(0o755)
    return str(directory) + os.pathsep + os.environ["PATH"]


@pytest.mark.skipif(
    sys.platform == "win32", reason="Uses a shell script as fake openscad"
)
def test_edit_shape_from_assets_directory(tmp_path):
    assets = tmp_path / "my robot" / "assets"
    assets.mkdir(parents=True)
    (assets / "part.stl").write_text("")
    code = (
        "import sys; sys.argv = ['edit-shape', 'part.stl'];"
        "from onshape_to_robot.edit_shape import main; main()"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=assets,
        capture_output=True,
        text=True,
        env=python_env(PATH=fake_openscad(tmp_path)),
    )
    assert f"RUN_IN={assets}" in result.stdout


def run_pure_sketch(
    tmp_path: Path, args: list, sketches: list = [], **env
) -> subprocess.CompletedProcess:
    """
    Runs onshape-to-robot-pure-sketch on assets/part.stl of a robot directory
    (tmp_path / "my robot"), with a fake Onshape client returning the given
    sketches
    """
    robot = tmp_path / "my robot"
    (robot / "assets").mkdir(parents=True)
    (robot / "config.json").write_text('{"document_id": "x", "output_format": "urdf"}')
    # An unrelated config.json higher in the tree is not used
    (tmp_path / "config.json").write_text("{}")
    (robot / "assets" / "part.stl").write_text("")
    (robot / "assets" / "part.part").write_text(
        json.dumps(
            {
                "partId": "p",
                "documentId": "d",
                "documentMicroversion": "m",
                "elementId": "e",
                "configuration": "default",
            }
        )
    )
    code = (
        f"import sys; sys.argv = ['pure-sketch'] + {args!r}\n"
        "from unittest import mock\n"
        "import onshape_to_robot.onshape_api.client as client\n"
        "class FakeClient:\n"
        "    def __init__(self, logging, creds): print('CREDS=' + creds)\n"
        f"    def get_sketches(self, *args): return {{'sketches': {sketches!r}}}\n"
        "with mock.patch.object(client, 'Client', FakeClient):\n"
        "    from onshape_to_robot.pure_sketch import main\n"
        "    main()\n"
    )
    return subprocess.run(
        [sys.executable, "-c", code],
        cwd=robot / "assets",
        capture_output=True,
        text=True,
        env=python_env(**env),
    )


def test_pure_sketch_finds_robot_config(tmp_path):
    result = run_pure_sketch(tmp_path, ["part.stl"])
    assert f"CREDS={tmp_path / 'my robot' / 'config.json'}" in result.stdout


def pure_shapes_sketch(name: str) -> dict:
    """A sketch with a circle and a rectangle (2 cm x 1 cm)"""
    rectangle = {
        "top.start": [0.0, 0.0],
        "top.end": [0.02, 0.0],
        "bottom.start": [0.0, 0.01],
        "bottom.end": [0.02, 0.01],
    }
    return {
        "sketch": name,
        "transformMatrix": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
        "geomEntities": [
            {"entityType": "circle", "id": "c", "center": [0.01, 0.02], "radius": 0.005}
        ]
        + [
            {"entityType": "point", "id": f"r.{key}", "point": point}
            for key, point in rectangle.items()
        ],
    }


@pytest.mark.skipif(
    sys.platform == "win32", reason="Uses a shell script as fake openscad"
)
def test_pure_sketch_generates_scad(tmp_path):
    result = run_pure_sketch(
        tmp_path,
        ["part.stl"],
        [pure_shapes_sketch("PureShapes 5.3"), pure_shapes_sketch("Other 2")],
        PATH=fake_openscad(tmp_path),
    )
    assets = tmp_path / "my robot" / "assets"
    assert f"RUN_IN={assets}" in result.stdout
    scad = (assets / "part.scad").read_text()
    assert 'import("part.stl")' in scad
    # Only the sketch with the prefix is used, extruded by its thickness
    assert scad.count("// Sketch") == 1
    assert "thickness = 5.300000;" in scad
    assert "translate([10.000000, 20.000000, 0])" in scad
    assert "cylinder(r=5.000000,h=thickness);" in scad
    assert "cube([20.000000, 10.000000, thickness]);" in scad


@pytest.mark.skipif(
    sys.platform == "win32", reason="Uses a shell script as fake openscad"
)
@pytest.mark.parametrize(
    "args", [["--quiet", "part.stl", "Custom"], ["part.stl", "Custom", "--quiet"]]
)
def test_pure_sketch_quiet(tmp_path, args):
    """With --quiet, the .scad file is written without opening OpenSCAD"""
    result = run_pure_sketch(
        tmp_path,
        args,
        [pure_shapes_sketch("Custom 3")],
        PATH=fake_openscad(tmp_path),
    )
    assert "RUN_IN=" not in result.stdout
    scad = (tmp_path / "my robot" / "assets" / "part.scad").read_text()
    assert "// Sketch Custom 3" in scad
