def main():
    import os
    import sys
    import pickle
    from dotenv import load_dotenv, find_dotenv
    from .config import Config
    from .message import error, info
    from .robot_builder import RobotBuilder
    from .exporter_urdf import ExporterURDF
    from .exporter_sdf import ExporterSDF
    from .exporter_mujoco import ExporterMuJoCo

    """
    This is the entry point of the export script, i.e the "onshape-to-robot" command.
    """
    load_dotenv(find_dotenv(usecwd=True))

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
    main()
