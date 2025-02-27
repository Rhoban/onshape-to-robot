import subprocess
import re
import json
import os
from .message import bright, info, error
from .processor import Processor
from .config import Config
from .robot import Robot
from .geometry import Box, Cylinder, Sphere, Shape
import numpy as np


class ProcessorScad(Processor):
    """
    Scad processor. This processor will parse OpenSCAD files to create pure shapes when available.

    The code is a naive parser of the intermediate CSG file produced by OpenSCAD, gathering Box, Sphere and Cylinders.
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # OpenSCAD pure shapes
        self.use_scads: bool = config.get("use_scads", False)
        self.pure_shape_dilatation: float = config.get("pure_shape_dilatation", 0.0)

        if self.use_scads:
            self.check_openscad()

    def check_openscad(self):
        if self.use_scads:
            print(bright("* Checking OpenSCAD presence..."))
            try:
                subprocess.run(["openscad", "-v"])
            except FileNotFoundError:
                print(bright("Can't run openscad -v, disabling OpenSCAD support"))
                print(info("TIP: consider installing openscad:"))
                print(info("Linux:"))
                print(info("sudo add-apt-repository ppa:openscad/releases"))
                print(info("sudo apt-get update"))
                print(info("sudo apt-get install openscad"))
                print(info("Windows:"))
                print(info("go to: https://openscad.org/downloads.html "))
                self.use_scads = False

    def process(self, robot: Robot):
        if self.use_scads:
            print(info("+ Parsing OpenSCAD files..."))
            for link in robot.links:
                for part in link.parts:
                    converted_meshes = []
                    for mesh in part.meshes:
                        if mesh.collision:
                            scad_file = mesh.filename.replace(".stl", ".scad")
                            if os.path.exists(scad_file):
                                part.shapes += self.parse_scad(scad_file, mesh.color)
                            converted_meshes.append(mesh)

                    for converted_mesh in converted_meshes:
                        converted_mesh.collision = False
                        part.prune_unused_geometry()

    def multmatrix_parse(self, parameters: str):
        matrix = np.matrix(json.loads(parameters), dtype=float)
        matrix[0, 3] /= 1000.0
        matrix[1, 3] /= 1000.0
        matrix[2, 3] /= 1000.0

        return matrix

    def cube_parse(self, parameters: str):
        results = re.findall(r"^size = (.+), center = (.+)$", parameters)
        if len(results) != 1:
            raise Exception(f"! Can't parse CSG cube parameters: {parameters}")
        extra = np.array([self.pure_shape_dilatation] * 3)

        return (
            extra + np.array(json.loads(results[0][0]), dtype=float) / 1000.0
        ), results[0][1] == "true"

    def cylinder_parse(self, parameters: str):
        results = re.findall(
            r"h = (.+), r1 = (.+), r2 = (.+), center = (.+)", parameters
        )
        if len(results) != 1:
            raise Exception(f"! Can't parse CSG cylinder parameters: {parameters}")
        result = results[0]
        extra = np.array([self.pure_shape_dilatation / 2, self.pure_shape_dilatation])

        return (extra + np.array([result[0], result[1]], dtype=float) / 1000.0), result[
            3
        ] == "true"

    def sphere_parse(self, parameters: str):
        results = re.findall(r"r = (.+)$", parameters)
        if len(results) != 1:
            raise Exception(f"! Can't parse CSG sphere parameters: {parameters}")

        return self.pure_shape_dilatation + float(results[0]) / 1000.0

    def extract_node_parameters(self, line: str):
        line = line.strip()
        parts = line.split("(", 1)
        node = parts[0]
        parameters = parts[1]
        if parameters[-1] == ";":
            parameters = parameters[:-2]
        if parameters[-1] == "{":
            parameters = parameters[:-3]
        return node, parameters

    def translation(self, x: float, y: float, z: float):
        m = np.eye(4)
        m[:3, 3] = [x, y, z]

        return m

    def parse_csg(self, csg_data: str, color):
        shapes: list[Shape] = []
        lines = csg_data.split("\n")
        matrices = []

        for line in lines:
            line = line.strip()
            if line != "":
                if line[-1] == "{":
                    node, parameters = self.extract_node_parameters(line)
                    if node == "multmatrix":
                        matrix = self.multmatrix_parse(parameters)
                    else:
                        matrix = np.eye(4)
                    matrices.append(matrix)
                elif line[-1] == "}":
                    matrices.pop()
                else:
                    node, parameters = self.extract_node_parameters(line)

                    transform = np.eye(4)
                    for matrix in matrices:
                        transform = transform @ matrix

                    if node == "cube":
                        size, center = self.cube_parse(parameters)
                        if not center:
                            transform = transform @ self.translation(
                                size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
                            )
                        shapes.append(
                            Box(transform, size, color, visual=False, collision=True)
                        )
                    if node == "cylinder":
                        size, center = self.cylinder_parse(parameters)
                        if not center:
                            transform = transform @ self.translation(
                                0, 0, size[0] / 2.0
                            )
                        shapes.append(
                            Cylinder(
                                transform,
                                size[0],
                                size[1],
                                color,
                                visual=False,
                                collision=True,
                            )
                        )
                    if node == "sphere":
                        shapes.append(
                            Sphere(
                                transform,
                                self.sphere_parse(parameters),
                                color,
                                visual=False,
                                collision=True,
                            )
                        )

        return shapes

    def parse_scad(self, scad_file: str, color: np.ndarray):
        tmp_data = os.getcwd() + "/_tmp_data.csg"
        os.system("openscad " + scad_file + " -o " + tmp_data)
        with open(tmp_data, "r", encoding="utf-8") as stream:
            data = stream.read()
        os.system("rm " + tmp_data)

        return self.parse_csg(data, color)
