def main():
    import os
    import sys

    if len(sys.argv) < 2:
        print("Usage: onshape-to-robot-edit-shape {STL file}")
    else:
        fileName = sys.argv[1]
        parts = fileName.split(".")
        parts[-1] = "scad"
        fileName = ".".join(parts)
        if not os.path.exists(fileName):
            scad = '% scale(1000) import("' + os.path.basename(sys.argv[1]) + '");\n'
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
    main()
