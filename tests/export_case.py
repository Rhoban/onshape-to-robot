"""
Run a full onshape-to-robot export of a test case (see cases.json), the same
way as the onshape-to-robot command does.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

CASES_FILE = Path(__file__).parent / "cases.json"
FORMATS = {"urdf": "urdf", "sdf": "sdf", "mujoco": "xml"}


def load_cases() -> list[dict]:
    with open(CASES_FILE, encoding="utf-8") as f:
        return json.load(f)["cases"]


def write_config(case: dict, output_format: str, directory: Path):
    config = {
        "document_id": case["document_id"],
        "version_id": case["version_id"],
        "output_format": output_format,
        "output_filename": "robot",
        **({"assembly_name": case["assembly"]} if case.get("assembly") else {}),
        **case.get("config", {}),
    }
    directory.mkdir(parents=True, exist_ok=True)
    with open(directory / "config.json", "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)


def export(case: dict, output_format: str, directory: Path) -> Path:
    """Export the case in directory, return the path of the main output file"""
    from onshape_to_robot.config import Config
    from onshape_to_robot.robot_builder import RobotBuilder
    from onshape_to_robot.exporter_urdf import ExporterURDF
    from onshape_to_robot.exporter_sdf import ExporterSDF
    from onshape_to_robot.exporter_mujoco import ExporterMuJoCo

    write_config(case, output_format, directory)
    cwd = os.getcwd()
    os.chdir(directory)
    try:
        config = Config(str(directory), safe=True)
        exporter = {
            "urdf": ExporterURDF,
            "sdf": ExporterSDF,
            "mujoco": ExporterMuJoCo,
        }[output_format](config)
        robot = RobotBuilder(config).robot
        for processor in config.processors:
            processor.process(robot)
        output = directory / f"robot.{exporter.ext}"
        exporter.write_xml(robot, str(output))
        return output
    finally:
        os.chdir(cwd)
