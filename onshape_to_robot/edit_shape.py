def main(stl_file):
    import os
    fileName = stl_file
    parts = fileName.split(".")
    parts[-1] = "scad"
    fileName = ".".join(parts)
    if not os.path.exists(fileName):
        scad = '% scale(1000) import("' + os.path.basename(stl_file) + '");\n'
        scad += "\n"
        scad += "// Append pure shapes (cube, cylinder and sphere), e.g:\n"
        scad += "// cube([10, 10, 10], center=true);\n"
        scad += "// cylinder(r=10, h=10, center=true);\n"
        scad += "// sphere(10);\n"
        with open(fileName, "w", encoding="utf-8") as stream:
            stream.write(scad)
    directory = os.path.dirname(fileName)
    os.system("cd " + directory + "; openscad " + os.path.basename(fileName))


if __name__ == "__main__":
    import argparse

    import argcomplete
    parser = argparse.ArgumentParser(
        prog="onshape-to-robot-edit-shape",
        description="Edit an STL file and create a SCAD template."
    )
    parser.add_argument("stl_file", help="Path to STL file")
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    main(args.stl_file)
