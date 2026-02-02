def main():
    import os
    import sys
    import pickle
    import argparse
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

    def get_version():
        # Get version from package
        try:
            from importlib.metadata import version, PackageNotFoundError

            return version("onshape-to-robot")
        except PackageNotFoundError:
            return "unknown"

    try:
        # Retrieving robot path
        arg_parser = argparse.ArgumentParser()
        arg_parser.add_argument(
            "robot_path", type=str, help="Path to the robot directory"
        )
        arg_parser.add_argument(
            "--version", action="version", version=f"onshape-to-robot {get_version()}"
        )
        arg_parser.add_argument(
            "--retrieve",
            action="store_true",
            help="Only retrieve data and produce robot.pkl",
        )
        arg_parser.add_argument(
            "--save-pickle",
            action="store_true",
            help="Save the robot data to robot.pkl",
        )
        arg_parser.add_argument(
            "--convert",
            action="store_true",
            help="Only convert robot.pkl to the desired format",
        )
        arg_parser.add_argument(
            "--safe",
            action="store_true",
            help="Disable features involving custom commands or imports",
        )
        args = arg_parser.parse_args()

        robot_path: str = args.robot_path

        # Loading configuration
        config = Config(robot_path, safe=args.safe)

        # Building exporter beforehand, so that the configuration gets checked
        if config.output_format == "urdf":
            exporter = ExporterURDF(config)
        elif config.output_format == "sdf":
            exporter = ExporterSDF(config)
        elif config.output_format == "mujoco":
            exporter = ExporterMuJoCo(config)
        else:
            raise Exception(f"Unsupported output format: {config.output_format}")

        if not args.convert:
            # Building the robot
            robot_builder = RobotBuilder(config)
            robot = robot_builder.robot

        # Can be used for debugging
        pkl_filename = config.output_directory + "/robot.pkl"
        if args.retrieve or args.save_pickle:
            pickle.dump(robot, open(pkl_filename, "wb"))
            print(info(f"* Robot data saved to {pkl_filename}"))

        if args.convert:
            print(info(f"* Loading robot data from {pkl_filename}"))
            robot = pickle.load(open(pkl_filename, "rb"))

        if not args.retrieve:
            # Applying processors
            for processor in config.processors:
                processor.process(robot)

            exporter.write_xml(
                robot,
                config.output_directory
                + "/"
                + config.output_filename
                + "."
                + exporter.ext,
            )

            if not args.safe:
                # Executing post-import commands
                for command in config.post_import_commands:
                    print(info(f"* Running command: {command}"))
                    os.system(command)

    except Exception as e:
        print(error(f"ERROR: {e}"))
        raise e


if __name__ == "__main__":
    main()
