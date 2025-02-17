import sys
from .config import Config
from .message import error
from .assembly import Assembly

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

    # Processing the assembly
    assembly = Assembly(config)

    print(assembly.body_instance(0))

except Exception as e:
    print(error(f"ERROR: {e}"))
    raise e
