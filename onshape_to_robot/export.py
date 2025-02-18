import sys
import pickle
from .config import Config
from .message import error
from .robot_builder import RobotBuilder
from .processors import processors
from .exporter_urdf import ExporterURDF
from .exporter_mujoco import ExporterMuJoCo

try:
    # Retrieving robot path
    if len(sys.argv) <= 1:
        raise Exception(
            "ERROR: usage: onshape-to-robot {robot_directory}\n"
            + "Read documentation at https://onshape-to-robot.readthedocs.io/"
        )

    robot_path: str = sys.argv[1]

    # Loading configuration
    config = Config(robot_path)

    # Initializing processors
    for index, class_ in enumerate(processors):
        processors[index] = class_(config)

    # Building exporter beforehand, so that the configuration gets checked
    if config.output_format == "urdf":
        exporter = ExporterURDF(config)
    elif config.output_format == "mujoco":
        exporter = ExporterMuJoCo(config)
    else:
        raise Exception(f"Unsupported output format: {config.output_format}")

    # Building the robot
    # robot_builder = RobotBuilder(config)
    # robot = robot_builder.robot

    # pickle.dump(robot, open("robot.pkl", "wb"))
    robot = pickle.load(open("robot.pkl", "rb"))

    # Applying processors
    for processor in processors:
        processor.process(robot)

    exporter.write_xml(robot, config.output_directory + "/robot." + exporter.ext)

except Exception as e:
    print(error(f"ERROR: {e}"))
    raise e
