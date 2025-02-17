import sys
from .config import Config
from .message import error
from .assembly import Assembly
from .robot_builder import RobotBuilder

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

    # Retrieving and processing the assembly
    assembly = Assembly(config)

    # Building the robot
    robot_builder = RobotBuilder(config, assembly)

except Exception as e:
    print(error(f"ERROR: {e}"))
    raise e
