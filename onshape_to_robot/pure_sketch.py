def main():
    import numpy as np
    import math
    import commentjson as json
    import os
    import sys, os
    from dotenv import load_dotenv, find_dotenv
    from colorama import Fore, Back, Style

    load_dotenv(find_dotenv(usecwd=True))

    if len(sys.argv) < 2:
        print("Usage: onshape-to-robot-pure-shape {STL file} [prefix=PureShapes]")
    else:
        fileName = sys.argv[1]
        robotDir = os.path.dirname(fileName)
        configFile = os.path.join(robotDir, "config.json")
        prefix = "PureShapes"
        if len(sys.argv) > 2:
            prefix = sys.argv[2]

        from .onshape_api.client import Client

        client = Client(logging=False, creds=configFile)

        parts = fileName.split(".")
        parts[-1] = "part"
        partFileName = ".".join(parts)
        parts[-1] = "scad"
        scadFileName = ".".join(parts)

        with open(partFileName, "r", encoding="utf-8") as stream:
            part = json.load(stream)
        partid = part["partId"]
        result = client.get_sketches(
            part["documentId"],
            part["documentMicroversion"],
            part["elementId"],
            part["configuration"],
        )

        scad = '% scale(1000) import("' + os.path.basename(fileName) + '");\n'

        sketchDatas = []
        for sketch in result["sketches"]:
            if sketch["sketch"].startswith(prefix):
                parts = sketch["sketch"].split(" ")
                if len(parts) >= 2:
                    sketch["thickness"] = float(parts[1])
                else:
                    print(
                        Fore.RED
                        + 'ERROR: The sketch name should contain extrusion size (e.g "PureShapes 5.3")'
                        + Style.RESET_ALL
                    )
                    exit(0)
                sketchDatas.append(sketch)

        if len(sketchDatas):
            print(
                Fore.GREEN
                + "* Found "
                + str(len(sketchDatas))
                + " PureShapes sketches"
                + Style.RESET_ALL
            )
            for sketchData in sketchDatas:
                # Retrieving sketch transform matrix
                m = sketchData["transformMatrix"]
                mm = [m[0:4], m[4:8], m[8:12], m[12:16]]
                mm[0][3] *= 1000
                mm[1][3] *= 1000
                mm[2][3] *= 1000
                scad += "\n"
                scad += "// Sketch " + sketchData["sketch"] + "\n"
                scad += "multmatrix(" + str(mm) + ") {" + "\n"
                scad += "thickness = %f;\n" % sketchData["thickness"]
                scad += "translate([0, 0, -thickness]) {\n"

                boxes = {}

                def boxSet(id, pointName, point):
                    if id not in boxes:
                        boxes[id] = {}
                    boxes[id][pointName] = point

                for entry in sketchData["geomEntities"]:
                    if entry["entityType"] == "circle":
                        center = entry["center"]
                        scad += "  translate([%f, %f, 0]) {\n" % (
                            center[0] * 1000,
                            center[1] * 1000,
                        )
                        scad += "    cylinder(r=%f,h=thickness);\n" % (
                            entry["radius"] * 1000
                        )
                        scad += "  }\n"
                    if entry["entityType"] == "point":
                        parts = entry["id"].split(".")
                        if len(parts) == 3:
                            if parts[1] == "top" and parts[2] == "start":
                                boxSet(parts[0], "A", entry["point"])
                            if parts[1] == "top" and parts[2] == "end":
                                boxSet(parts[0], "B", entry["point"])
                            if parts[1] == "bottom" and parts[2] == "start":
                                boxSet(parts[0], "C", entry["point"])
                            if parts[1] == "bottom" and parts[2] == "end":
                                boxSet(parts[0], "D", entry["point"])

                for id in boxes:
                    if len(boxes[id]) == 4:
                        A, B = np.array(boxes[id]["A"]), np.array(boxes[id]["B"])
                        C, D = np.array(boxes[id]["C"]), np.array(boxes[id]["D"])
                        AB = B - A

                        # Making sure that the orientation of the square is correct
                        AB90 = np.array([-AB[1], AB[0]])
                        side = AB90.dot(C - A)
                        width = np.linalg.norm(B - A)
                        height = np.linalg.norm(B - D)
                        if side < 0:
                            A, B, C, D = C, D, A, B

                        AB = B - A
                        alpha = np.rad2deg(math.atan2(AB[1], AB[0]))
                        scad += "  translate([%f, %f, 0]) {\n" % (
                            A[0] * 1000,
                            A[1] * 1000,
                        )
                        scad += "    rotate([0, 0, " + str(alpha) + "]) {" + "\n"
                        scad += "      cube([%f, %f, thickness]);\n" % (
                            width * 1000,
                            height * 1000,
                        )
                        scad += "    }\n"
                        scad += "  }\n"

                scad += "}\n"
                scad += "}\n"

            with open(scadFileName, "w", encoding="utf-8") as stream:
                stream.write(scad)

            directory = os.path.dirname(fileName)
            os.system(
                "cd " + directory + "; openscad " + os.path.basename(scadFileName)
            )
        else:
            print(
                Fore.RED
                + "ERROR: Can't find pure shape sketch in this part"
                + Style.RESET_ALL
            )


if __name__ == "__main__":
    main()
