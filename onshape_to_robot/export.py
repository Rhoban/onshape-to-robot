def main(robot_directory=None):
    import os
    import pickle

    from dotenv import find_dotenv, load_dotenv

    from .config import Config
    from .exporter_mujoco import ExporterMuJoCo
    from .exporter_sdf import ExporterSDF
    from .exporter_urdf import ExporterURDF
    from .message import error, info
    from .robot_builder import RobotBuilder

    """
    This is the entry point of the export script, i.e the "onshape-to-robot" command.
    """
    load_dotenv(find_dotenv(usecwd=True))

    try:
        #retrieving robot path
        robot_path: str = robot_directory

        # Loading configuration
        config = Config(robot_path)

        # Building exporter beforehand, so that the configuration gets checked
        if config.output_format == "urdf":
            exporter = ExporterURDF(config)
        elif config.output_format == "sdf":
            exporter = ExporterSDF(config)
        elif config.output_format == "mujoco":
            exporter = ExporterMuJoCo(config)
        else:
            raise Exception(f"Unsupported output format: {config.output_format}")

        # Building the robot
        robot_builder = RobotBuilder(config)
        robot = robot_builder.robot

        # Can be used for debugging
        # pickle.dump(robot, open("robot.pkl", "wb"))
        # robot = pickle.load(open("robot.pkl", "rb"))

        # Applying processors
        for processor in config.processors:
            processor.process(robot)

        exporter.write_xml(
            robot,
            config.output_directory + "/" + config.output_filename + "." + exporter.ext,
        )

        for command in config.post_import_commands:
            print(info(f"* Running command: {command}"))
            os.system(command)

    except Exception as e:
        print(error(f"ERROR: {e}"))
        raise e


if __name__ == "__main__":
    import argparse

    import argcomplete

    parser = argparse.ArgumentParser(
    prog="onshape-to-robot",
        description="Convert Onshape assembly to robot definition (URDF, SDF, MuJoCo) through Onshape API."
    )
    parser.add_argument("robot_directory", help="Path to robot directory")
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    robot_directory = args.robot_directory
