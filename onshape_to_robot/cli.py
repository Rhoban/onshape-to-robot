import argparse
import sys

import argcomplete

from . import bullet, clear_cache, edit_shape, export, mujoco, pure_sketch


def main():
    print("onshape-to-robot command line interface")
    parser = argparse.ArgumentParser(
        prog="onshape-to-robot",
        description="Unified entry point for Onshape-to-Robot tools. Use a subcommand to access each tool."
    )
    subparsers = parser.add_subparsers(dest="command", help="Tool to run")

    # Export
    export_parser = subparsers.add_parser(
        "export",
        help="Convert Onshape assembly to robot definition (URDF, SDF, MuJoCo)"
    )
    export_parser.add_argument("robot_directory", help="Path to robot directory")
    export_parser.set_defaults(func=lambda args: export.main(args.robot_directory))

    # Bullet
    bullet_parser = subparsers.add_parser(
        "bullet",
        help="Run the PyBullet simulation for a URDF robot"
    )
    bullet_parser.add_argument("robot_directory", help="Path to robot directory")
    bullet_parser.add_argument("-f", "--fixed", action="store_true", help="Fix the robot base")
    bullet_parser.add_argument("-n", "--no-self-collisions", action="store_true", help="Ignore self collisions")
    bullet_parser.add_argument("-x", type=float, default=0, help="X position offset")
    bullet_parser.add_argument("-y", type=float, default=0, help="Y position offset")
    bullet_parser.add_argument("-z", type=float, default=0, help="Z position offset")
    bullet_parser.set_defaults(func=lambda args: sys.exit(
        bullet.main(
            args.robot_directory,
            fixed=args.fixed,
            no_self_collisions=args.no_self_collisions,
            x=args.x,
            y=args.y,
            z=args.z
        )
    ))

    # Mujoco
    mujoco_parser = subparsers.add_parser(
        "mujoco",
        help="Run the MuJoCo simulation for a robot"
    )
    mujoco_parser.add_argument("--sim", action="store_true", help="Enable simulation mode")
    mujoco_parser.add_argument("-x", type=float, default=0, help="X position offset")
    mujoco_parser.add_argument("-y", type=float, default=0, help="Y position offset")
    mujoco_parser.add_argument("-z", type=float, default=0.5, help="Z position offset")
    mujoco_parser.add_argument("robot_directory", help="Path to robot directory")
    mujoco_parser.set_defaults(func=lambda args: sys.exit(
        mujoco.main(
            args.sim,
            args.x,
            args.y,
            args.z,
            args.robot_directory
        )
    ))

    # Clear cache
    clear_cache_parser = subparsers.add_parser(
        "clear-cache",
        help="Clear the onshape-to-robot cache"
    )
    clear_cache_parser.set_defaults(func=lambda args: clear_cache.main())

    # Edit shape
    edit_shape_parser = subparsers.add_parser(
        "edit-shape",
        help="Edit an STL file and create a SCAD template"
    )
    edit_shape_parser.add_argument("stl_file", help="Path to STL file")
    edit_shape_parser.add_argument("output_file", help="Path to output SCAD file")
    edit_shape_parser.set_defaults(func=lambda args: sys.exit(
        edit_shape.main(args.stl_file, args.output_file)
    ))

    # Pure sketch
    pure_sketch_parser = subparsers.add_parser(
        "pure-sketch",
        help="Extract pure shapes from an STL file"
    )
    pure_sketch_parser.add_argument("stl_file", help="Path to STL file")
    pure_sketch_parser.add_argument("output_file", help="Path to output file")
    pure_sketch_parser.set_defaults(func=lambda args: sys.exit(
        pure_sketch.main(args.stl_file, args.output_file)
    ))

    argcomplete.autocomplete(parser)


    args = parser.parse_args()
    if hasattr(args, 'func'):
        args.func(args)
    else:
        parser.print_help()
        sys.exit(1)

if __name__ == "__main__":
    main()
